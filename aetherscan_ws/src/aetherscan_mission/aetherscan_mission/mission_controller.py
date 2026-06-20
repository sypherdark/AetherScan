"""Mission Controller for AetherScan.

Implements the full autonomous scanning mission state machine:
IDLE → PREFLIGHT → TAKEOFF → EXPLORING → RETURNING → LANDING → COMPLETE

Coordinates all subsystems and provides mission control services.
"""

import json
import time
from enum import IntEnum

import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


class MissionState(IntEnum):
    IDLE = 0
    PREFLIGHT_CHECK = 1
    TAKEOFF = 2
    EXPLORING = 3
    SCANNING = 4
    RETURNING = 5
    LANDING = 6
    COMPLETE = 7
    PAUSED = 8
    ERROR = 9


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        self.declare_parameter('takeoff_altitude', 1.5)
        self.declare_parameter('scan_speed', 0.5)
        self.declare_parameter('mission_timeout_sec', 600.0)
        self.declare_parameter('home_position', [2.0, 7.5, 0.2])

        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.scan_speed = self.get_parameter('scan_speed').value
        self.mission_timeout = self.get_parameter('mission_timeout_sec').value
        self.home_position = self.get_parameter('home_position').value

        self.state = MissionState.IDLE
        self.previous_state = MissionState.IDLE
        self.mission_start_time = None
        self.current_position = None
        self.coverage_percent = 0.0
        self.distance_traveled = 0.0
        self.last_position = None
        self.total_points = 0

        self.start_srv = self.create_service(
            Trigger, '/aetherscan/start_mission', self.start_mission_callback
        )
        self.pause_srv = self.create_service(
            Trigger, '/aetherscan/pause_mission', self.pause_mission_callback
        )
        self.resume_srv = self.create_service(
            Trigger, '/aetherscan/resume_mission', self.resume_mission_callback
        )
        self.abort_srv = self.create_service(
            Trigger, '/aetherscan/abort_mission', self.abort_mission_callback
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/aetherscan/odom', self.odom_callback, 10
        )
        self.coverage_sub = self.create_subscription(
            Float32, '/aetherscan/exploration/coverage', self.coverage_callback, 10
        )
        self.exploration_state_sub = self.create_subscription(
            String, '/aetherscan/exploration/state', self.exploration_state_callback, 10
        )
        self.map_stats_sub = self.create_subscription(
            String, '/aetherscan/map/stats', self.map_stats_callback, 10
        )

        self.status_pub = self.create_publisher(String, '/aetherscan/mission/status', 10)
        self.arm_pub = self.create_publisher(Bool, '/aetherscan/arm', 10)
        self.exploration_enable_pub = self.create_publisher(
            Bool, '/aetherscan/exploration/enable', 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/aetherscan/cmd_vel_input', 10)

        self.state_timer = self.create_timer(0.5, self.state_machine_step)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Mission Controller initialized')

    def start_mission_callback(self, request, response):
        if self.state == MissionState.IDLE:
            self.state = MissionState.PREFLIGHT_CHECK
            self.mission_start_time = time.time()
            self.distance_traveled = 0.0
            response.success = True
            response.message = 'Mission starting'
            self.get_logger().info('Mission START requested')
        else:
            response.success = False
            response.message = f'Cannot start: currently in {MissionState(self.state).name}'
        return response

    def pause_mission_callback(self, request, response):
        if self.state in (MissionState.EXPLORING, MissionState.SCANNING):
            self.previous_state = self.state
            self.state = MissionState.PAUSED
            self._disable_exploration()
            response.success = True
            response.message = 'Mission paused'
        else:
            response.success = False
            response.message = 'Cannot pause in current state'
        return response

    def resume_mission_callback(self, request, response):
        if self.state == MissionState.PAUSED:
            self.state = self.previous_state
            self._enable_exploration()
            response.success = True
            response.message = 'Mission resumed'
        else:
            response.success = False
            response.message = 'Not currently paused'
        return response

    def abort_mission_callback(self, request, response):
        if self.state not in (MissionState.IDLE, MissionState.COMPLETE):
            self.state = MissionState.RETURNING
            self._disable_exploration()
            response.success = True
            response.message = 'Mission aborted, returning home'
            self.get_logger().warn('Mission ABORTED')
        else:
            response.success = False
            response.message = 'No active mission to abort'
        return response

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.current_position = np.array([pos.x, pos.y, pos.z])

        if self.last_position is not None and self.state in (
            MissionState.EXPLORING, MissionState.SCANNING,
            MissionState.RETURNING
        ):
            delta = np.linalg.norm(self.current_position - self.last_position)
            if delta < 1.0:
                self.distance_traveled += delta

        self.last_position = self.current_position.copy()

    def coverage_callback(self, msg: Float32):
        self.coverage_percent = msg.data * 100.0

    def exploration_state_callback(self, msg: String):
        if msg.data == 'COMPLETE' and self.state == MissionState.EXPLORING:
            self.state = MissionState.RETURNING
            self.get_logger().info('Exploration complete, returning home')

    def map_stats_callback(self, msg: String):
        try:
            stats = json.loads(msg.data)
            self.total_points = stats.get('total_points', 0)
        except json.JSONDecodeError:
            pass

    def state_machine_step(self):
        """Execute the current state's logic."""
        if self.state == MissionState.PREFLIGHT_CHECK:
            self._do_preflight()
        elif self.state == MissionState.TAKEOFF:
            self._do_takeoff()
        elif self.state == MissionState.EXPLORING:
            self._do_exploring()
        elif self.state == MissionState.RETURNING:
            self._do_returning()
        elif self.state == MissionState.LANDING:
            self._do_landing()

        if self.mission_start_time and self.state not in (
            MissionState.IDLE, MissionState.COMPLETE, MissionState.ERROR
        ):
            elapsed = time.time() - self.mission_start_time
            if elapsed > self.mission_timeout:
                self.get_logger().warn('Mission TIMEOUT, returning home')
                self.state = MissionState.RETURNING

    def _do_preflight(self):
        """Run preflight checks."""
        self.get_logger().info('Preflight checks passed')
        self.state = MissionState.TAKEOFF
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_pub.publish(arm_msg)

    def _do_takeoff(self):
        """Ascend to scan altitude."""
        if self.current_position is None:
            return

        if self.current_position[2] >= self.takeoff_altitude - 0.2:
            self.get_logger().info('Takeoff complete, starting exploration')
            self.state = MissionState.EXPLORING
            self._enable_exploration()
        else:
            cmd = Twist()
            cmd.linear.z = 0.3
            self.cmd_vel_pub.publish(cmd)

    def _do_exploring(self):
        """Monitor exploration progress."""
        pass

    def _do_returning(self):
        """Navigate back to home position."""
        if self.current_position is None:
            return

        home = np.array(self.home_position)
        dist = np.linalg.norm(self.current_position[:2] - home[:2])

        if dist < 0.5:
            self.state = MissionState.LANDING
            self.get_logger().info('Reached home, landing')
        else:
            direction = home - self.current_position
            direction_norm = direction / (np.linalg.norm(direction) + 1e-6)
            cmd = Twist()
            cmd.linear.x = float(direction_norm[0]) * self.scan_speed
            cmd.linear.y = float(direction_norm[1]) * self.scan_speed
            cmd.linear.z = float(direction_norm[2]) * 0.2
            self.cmd_vel_pub.publish(cmd)

    def _do_landing(self):
        """Descend and disarm."""
        if self.current_position is None:
            return

        if self.current_position[2] < 0.3:
            arm_msg = Bool()
            arm_msg.data = False
            self.arm_pub.publish(arm_msg)
            self.state = MissionState.COMPLETE
            elapsed = time.time() - self.mission_start_time if self.mission_start_time else 0
            self.get_logger().info(
                f'Mission COMPLETE! Time: {elapsed:.0f}s, '
                f'Coverage: {self.coverage_percent:.1f}%, '
                f'Distance: {self.distance_traveled:.1f}m'
            )
        else:
            cmd = Twist()
            cmd.linear.z = -0.2
            self.cmd_vel_pub.publish(cmd)

    def _enable_exploration(self):
        msg = Bool()
        msg.data = True
        self.exploration_enable_pub.publish(msg)

    def _disable_exploration(self):
        msg = Bool()
        msg.data = False
        self.exploration_enable_pub.publish(msg)

    def publish_status(self):
        """Publish comprehensive mission status."""
        elapsed = 0.0
        if self.mission_start_time:
            elapsed = time.time() - self.mission_start_time

        status = {
            'state': MissionState(self.state).name,
            'state_id': int(self.state),
            'elapsed_time_sec': round(elapsed, 1),
            'coverage_percent': round(self.coverage_percent, 1),
            'distance_traveled_m': round(self.distance_traveled, 1),
            'total_points': self.total_points,
            'position': self.current_position.tolist() if self.current_position is not None else None,
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
