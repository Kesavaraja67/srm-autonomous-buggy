#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

MAX_LINEAR_SPEED = 4.17
RESUME_RAMP_DUR  = 1.0

class SpeedControllerNode(Node):
    def __init__(self):
        super().__init__('speed_controller_node')
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/buggy_state',  self._state_cb,      10)
        self.create_subscription(Twist,  '/nav_cmd_vel',  self._nav_cmd_vel_cb, 10)
        self._current_state     = 'WAITING_FOR_DESTINATION'
        self._nav_cmd           = Twist()
        self._resume_start_time = 0.0
        self.create_timer(0.1, self._control_loop)
        self.get_logger().info('SpeedControllerNode started. Only publisher of /cmd_vel.')

    def _state_cb(self, msg):
        prev = self._current_state
        self._current_state = msg.data
        if self._current_state == 'RESUMING' and prev != 'RESUMING':
            self._resume_start_time = time.monotonic()

    def _nav_cmd_vel_cb(self, msg):
        self._nav_cmd = msg

    def _control_loop(self):
        out = Twist()
        s = self._current_state
        if s == 'NAVIGATING':
            out = self._nav_cmd
        elif s == 'RESUMING':
            elapsed = time.monotonic() - self._resume_start_time
            ratio   = min(1.0, elapsed / RESUME_RAMP_DUR)
            out.linear.x  = self._nav_cmd.linear.x * ratio
            out.angular.z = self._nav_cmd.angular.z
        self._cmd_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = SpeedControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()