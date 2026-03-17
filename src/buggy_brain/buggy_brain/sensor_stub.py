#!/usr/bin/env python3
"""
sensor_stub.py  --  Team Bravo SITL Stub
-----------------------------------------
Simulates Team Charlie's sensor nodes for standalone testing.
Publishes safe (False) values on all sensor topics so the
state machine stays in NAVIGATING without Team Charlie's
real lidar / ultrasonic / crowd-detection code.

Topics published (all std_msgs/Bool, latched, 1 Hz):
  /ultrasonic_alert   → False  (Ultrasonic safe)
  /crowd_detected     → False  (No crowd)
  /manual_override    → False  (Not in manual mode)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SensorStubNode(Node):
    def __init__(self):
        super().__init__('sensor_stub_node')

        RATE = 1.0  # Hz — one "all-clear" message per second is enough

        self._pubs = {
            '/ultrasonic_alert':  self.create_publisher(Bool, '/ultrasonic_alert',  10),
            '/crowd_detected':    self.create_publisher(Bool, '/crowd_detected',     10),
            '/manual_override':   self.create_publisher(Bool, '/manual_override',    10),
        }
        self.create_timer(1.0 / RATE, self._publish)
        self.get_logger().info(
            'SensorStubNode started — publishing all-clear on sensor topics.'
        )

    def _publish(self):
        msg = Bool()
        msg.data = False
        for pub in self._pubs.values():
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorStubNode()
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
