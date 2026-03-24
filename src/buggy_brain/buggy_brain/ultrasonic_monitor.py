import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String


class UltrasonicMonitor(Node):
    """
    ROS 2 node that monitors 4 ultrasonic sensors and publishes
    safety stop/clear commands as a redundant safety layer.

    Subscribes : /ultrasonic/front  (sensor_msgs/Range)
                 /ultrasonic/rear   (sensor_msgs/Range)
                 /ultrasonic/left   (sensor_msgs/Range)
                 /ultrasonic/right  (sensor_msgs/Range)
    Publishes  : /safety_status     (std_msgs/String)
                 /ultrasonic_alert  (std_msgs/String)
    """

    def __init__(self):
        super().__init__('ultrasonic_monitor')

        # --- Parameters ---
        self.stop_distance  = 0.3   # metres — stop if closer than this
        self.clear_distance = 0.6   # metres — resume when beyond this

        # --- State per sensor ---
        self.sensor_states = {
            'front': False,
            'rear':  False,
            'left':  False,
            'right': False,
        }

        # --- Subscribers ---
        self.create_subscription(
            Range, '/ultrasonic/front',
            lambda msg: self.sensor_callback(msg, 'front'), 10)
        self.create_subscription(
            Range, '/ultrasonic/rear',
            lambda msg: self.sensor_callback(msg, 'rear'), 10)
        self.create_subscription(
            Range, '/ultrasonic/left',
            lambda msg: self.sensor_callback(msg, 'left'), 10)
        self.create_subscription(
            Range, '/ultrasonic/right',
            lambda msg: self.sensor_callback(msg, 'right'), 10)

        # --- Publishers ---
        self.safety_pub = self.create_publisher(
            String, '/safety_status', 10)
        self.alert_pub  = self.create_publisher(
            String, '/ultrasonic_alert', 10)

        # --- Status timer at 10 Hz ---
        self.create_timer(0.1, self.safety_loop)

        self.get_logger().info('Ultrasonic monitor started.')
        self.get_logger().info(
            f'Stop: {self.stop_distance}m | '
            f'Clear: {self.clear_distance}m')

    def sensor_callback(self, msg, sensor_name):
        """Process each ultrasonic sensor reading."""
        distance = msg.range

        if distance < self.stop_distance:
            if not self.sensor_states[sensor_name]:
                self.get_logger().warn(
                    f'ULTRASONIC [{sensor_name.upper()}]: '
                    f'obstacle at {distance:.2f}m — STOP')
                self._publish_alert(
                    f'ULTRASONIC_{sensor_name.upper()}: {distance:.2f}m')
            self.sensor_states[sensor_name] = True

        elif distance > self.clear_distance:
            if self.sensor_states[sensor_name]:
                self.get_logger().info(
                    f'ULTRASONIC [{sensor_name.upper()}]: clear')
            self.sensor_states[sensor_name] = False

    def safety_loop(self):
        """Publish STOP if any sensor is triggered, CLEAR if all clear."""
        any_triggered = any(self.sensor_states.values())

        msg      = String()
        msg.data = 'STOP' if any_triggered else 'CLEAR'
        self.safety_pub.publish(msg)

    def _publish_alert(self, text):
        msg      = String()
        msg.data = text
        self.alert_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
