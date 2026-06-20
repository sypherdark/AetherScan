"""Flight Controller for AetherScan.

Implements cascaded PID position/velocity controller that accepts position
setpoints or velocity commands and outputs velocity commands for Gazebo.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class PIDController:
    """Single-axis PID controller with anti-windup."""

    __slots__ = ['kp', 'ki', 'kd', 'integral', 'prev_error',
                 'max_integral', 'output_limit']

    def __init__(self, kp: float, ki: float, kd: float,
                 max_integral: float = 5.0, output_limit: float = 1.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.max_integral = max_integral
        self.output_limit = output_limit

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.max_integral, self.max_integral)

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return np.clip(output, -self.output_limit, self.output_limit)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class FlightController(Node):
    def __init__(self):
        super().__init__('flight_controller')

        self.declare_parameter('pos_kp', 1.2)
        self.declare_parameter('pos_ki', 0.05)
        self.declare_parameter('pos_kd', 0.4)
        self.declare_parameter('alt_kp', 1.5)
        self.declare_parameter('alt_ki', 0.1)
        self.declare_parameter('alt_kd', 0.5)
        self.declare_parameter('yaw_kp', 1.0)
        self.declare_parameter('yaw_ki', 0.0)
        self.declare_parameter('yaw_kd', 0.2)
        self.declare_parameter('max_horizontal_vel', 1.5)
        self.declare_parameter('max_vertical_vel', 0.5)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('position_tolerance', 0.1)

        self.pid_x = PIDController(
            self.get_parameter('pos_kp').value,
            self.get_parameter('pos_ki').value,
            self.get_parameter('pos_kd').value,
            output_limit=self.get_parameter('max_horizontal_vel').value
        )
        self.pid_y = PIDController(
            self.get_parameter('pos_kp').value,
            self.get_parameter('pos_ki').value,
            self.get_parameter('pos_kd').value,
            output_limit=self.get_parameter('max_horizontal_vel').value
        )
        self.pid_z = PIDController(
            self.get_parameter('alt_kp').value,
            self.get_parameter('alt_ki').value,
            self.get_parameter('alt_kd').value,
            output_limit=self.get_parameter('max_vertical_vel').value
        )
        self.pid_yaw = PIDController(
            self.get_parameter('yaw_kp').value,
            self.get_parameter('yaw_ki').value,
            self.get_parameter('yaw_kd').value,
            output_limit=self.get_parameter('max_yaw_rate').value
        )

        self.current_position = None
        self.current_yaw = 0.0
        self.target_position = None
        self.target_yaw = 0.0
        self.mode = 'IDLE'  # IDLE, POSITION, VELOCITY
        self.armed = False
        self.last_time = self.get_clock().now()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/aetherscan/odom', self.odom_callback, sensor_qos
        )
        self.setpoint_sub = self.create_subscription(
            PoseStamped, '/aetherscan/navigation/current_waypoint',
            self.setpoint_callback, 10
        )
        self.vel_cmd_sub = self.create_subscription(
            Twist, '/aetherscan/cmd_vel_input', self.velocity_callback, 10
        )
        self.arm_sub = self.create_subscription(
            Bool, '/aetherscan/arm', self.arm_callback, 10
        )

        self.cmd_pub = self.create_publisher(
            Twist, '/aetherscan/cmd_vel_raw', 10
        )
        self.status_pub = self.create_publisher(
            String, '/aetherscan/controller/status', 10
        )

        control_rate = self.get_parameter('control_rate').value
        self.control_timer = self.create_timer(1.0 / control_rate, self.control_loop)

        self.get_logger().info('Flight Controller initialized')

    def arm_callback(self, msg: Bool):
        self.armed = msg.data
        if self.armed:
            self.get_logger().info('Flight controller ARMED')
        else:
            self.get_logger().info('Flight controller DISARMED')
            self.mode = 'IDLE'

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.current_position = np.array([pos.x, pos.y, pos.z])

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def setpoint_callback(self, msg: PoseStamped):
        """Receive position setpoint from path planner."""
        if not self.armed:
            return

        self.target_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        q = msg.pose.orientation
        if abs(q.w) > 0.001:
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.target_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.mode = 'POSITION'

    def velocity_callback(self, msg: Twist):
        """Direct velocity command (from teleop)."""
        if not self.armed:
            return
        self.mode = 'VELOCITY'
        self.cmd_pub.publish(msg)

    def control_loop(self):
        """Main control loop at fixed rate."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        status_msg = String()
        status_msg.data = f'{self.mode}|armed={self.armed}'
        self.status_pub.publish(status_msg)

        if not self.armed or self.mode != 'POSITION':
            return

        if self.current_position is None or self.target_position is None:
            return

        error = self.target_position - self.current_position

        cmd = Twist()
        cmd.linear.x = self.pid_x.compute(error[0], dt)
        cmd.linear.y = self.pid_y.compute(error[1], dt)
        cmd.linear.z = self.pid_z.compute(error[2], dt)

        yaw_error = self.target_yaw - self.current_yaw
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        cmd.angular.z = self.pid_yaw.compute(yaw_error, dt)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FlightController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
