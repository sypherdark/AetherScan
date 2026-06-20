"""Keyboard Teleoperation for AetherScan drone.

Provides keyboard-based manual control with real-time velocity
feedback and mode switching capabilities.
"""

import sys
import select
import termios
import tty
from threading import Thread

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

HELP_TEXT = """
╔══════════════════════════════════════════════════════╗
║           AetherScan Keyboard Teleop                 ║
╠══════════════════════════════════════════════════════╣
║                                                      ║
║  Movement:          Altitude:        Yaw:            ║
║    W - Forward        R - Up           Q - Left      ║
║    S - Backward       F - Down         E - Right     ║
║    A - Strafe Left                                   ║
║    D - Strafe Right                                  ║
║                                                      ║
║  Commands:                                           ║
║    T - Takeoff (arm + altitude)                      ║
║    L - Land (descend + disarm)                       ║
║    SPACE - Emergency Stop                            ║
║    M - Toggle Autonomous Mode                        ║
║    X - Disarm                                        ║
║    ESC/Ctrl+C - Quit                                 ║
║                                                      ║
║  Speed: +/- to adjust                                ║
║                                                      ║
╚══════════════════════════════════════════════════════╝
"""

KEY_BINDINGS = {
    'w': (1.0, 0.0, 0.0, 0.0),   # forward
    's': (-1.0, 0.0, 0.0, 0.0),  # backward
    'a': (0.0, 1.0, 0.0, 0.0),   # strafe left
    'd': (0.0, -1.0, 0.0, 0.0),  # strafe right
    'r': (0.0, 0.0, 1.0, 0.0),   # up
    'f': (0.0, 0.0, -1.0, 0.0),  # down
    'q': (0.0, 0.0, 0.0, 1.0),   # yaw left
    'e': (0.0, 0.0, 0.0, -1.0),  # yaw right
}


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('vertical_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('takeoff_altitude', 1.5)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.vertical_speed = self.get_parameter('vertical_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value

        self.cmd_pub = self.create_publisher(Twist, '/aetherscan/cmd_vel_input', 10)
        self.arm_pub = self.create_publisher(Bool, '/aetherscan/arm', 10)
        self.mode_pub = self.create_publisher(Bool, '/aetherscan/exploration/enable', 10)

        self.armed = False
        self.autonomous_mode = False
        self.running = True

        self.get_logger().info('Keyboard teleop ready')

    def run(self):
        """Main loop reading keyboard input."""
        print(HELP_TEXT)
        print(f"Speed: linear={self.linear_speed:.1f} vertical={self.vertical_speed:.1f}")
        print(f"Status: {'ARMED' if self.armed else 'DISARMED'} | Mode: {'AUTO' if self.autonomous_mode else 'MANUAL'}\n")

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while self.running and rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    self._handle_key(key)
                else:
                    cmd = Twist()
                    self.cmd_pub.publish(cmd)
        except Exception as e:
            self.get_logger().error(f'Teleop error: {e}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)

    def _handle_key(self, key: str):
        """Process a single key press."""
        if key == '\x1b' or key == '\x03':
            self.running = False
            print("\nShutting down teleop...")
            return

        if key == ' ':
            self._emergency_stop()
            return

        if key == 't':
            self._takeoff()
            return

        if key == 'l':
            self._land()
            return

        if key == 'm':
            self._toggle_autonomous()
            return

        if key == 'x':
            self._disarm()
            return

        if key == '+' or key == '=':
            self.linear_speed = min(2.0, self.linear_speed + 0.1)
            self.vertical_speed = min(1.0, self.vertical_speed + 0.05)
            self._print_status()
            return

        if key == '-':
            self.linear_speed = max(0.1, self.linear_speed - 0.1)
            self.vertical_speed = max(0.05, self.vertical_speed - 0.05)
            self._print_status()
            return

        if key in KEY_BINDINGS:
            vx, vy, vz, wz = KEY_BINDINGS[key]
            cmd = Twist()
            cmd.linear.x = vx * self.linear_speed
            cmd.linear.y = vy * self.linear_speed
            cmd.linear.z = vz * self.vertical_speed
            cmd.angular.z = wz * self.angular_speed
            self.cmd_pub.publish(cmd)
            self._print_vel(cmd)

    def _takeoff(self):
        """Arm and command takeoff altitude."""
        self.armed = True
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_pub.publish(arm_msg)

        cmd = Twist()
        cmd.linear.z = self.vertical_speed
        self.cmd_pub.publish(cmd)
        print(f"\r  >> TAKEOFF (armed, ascending to {self.takeoff_altitude}m)    ")

    def _land(self):
        """Command landing."""
        cmd = Twist()
        cmd.linear.z = -self.vertical_speed * 0.5
        self.cmd_pub.publish(cmd)
        print("\r  >> LANDING...                              ")

    def _emergency_stop(self):
        """Emergency stop - zero all velocities."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        print("\r  !! EMERGENCY STOP !!                       ")

    def _disarm(self):
        """Disarm the drone."""
        self.armed = False
        arm_msg = Bool()
        arm_msg.data = False
        self.arm_pub.publish(arm_msg)
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        print("\r  >> DISARMED                                ")

    def _toggle_autonomous(self):
        """Toggle autonomous exploration mode."""
        self.autonomous_mode = not self.autonomous_mode
        mode_msg = Bool()
        mode_msg.data = self.autonomous_mode
        self.mode_pub.publish(mode_msg)
        mode_str = "AUTONOMOUS" if self.autonomous_mode else "MANUAL"
        print(f"\r  >> Mode: {mode_str}                       ")

    def _print_vel(self, cmd: Twist):
        """Print current velocity command."""
        print(f"\r  vel: x={cmd.linear.x:+.2f} y={cmd.linear.y:+.2f} "
              f"z={cmd.linear.z:+.2f} yaw={cmd.angular.z:+.2f}    ", end='')

    def _print_status(self):
        """Print current speed settings."""
        print(f"\r  Speed: linear={self.linear_speed:.1f} "
              f"vertical={self.vertical_speed:.2f}         ", end='')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
