import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StateMachine(Node):
    """
    ROS 2 node that manages overall buggy behaviour state.

    States:
        IDLE       — waiting for a goal
        PLANNING   — path planner is computing a route
        NAVIGATING — waypoint follower is driving the buggy
        STOPPED    — obstacle detected, waiting for clear
        ARRIVED    — destination reached
        ERROR      — something went wrong

    Subscribes : /planning_status  (std_msgs/String)
                 /waypoint_status  (std_msgs/String)
                 /safety_status    (std_msgs/String)
                 /goal_destination (std_msgs/String)
    Publishes  : /buggy_state      (std_msgs/String)
    """

    IDLE       = 'IDLE'
    PLANNING   = 'PLANNING'
    NAVIGATING = 'NAVIGATING'
    STOPPED    = 'STOPPED'
    ARRIVED    = 'ARRIVED'
    ERROR      = 'ERROR'

    def __init__(self):
        super().__init__('state_machine')

        self.state = self.IDLE
        self.previous_state = None

        # --- Subscribers ---
        self.create_subscription(
            String, '/goal_destination',
            self.goal_callback, 10)

        self.create_subscription(
            String, '/planning_status',
            self.planning_callback, 10)

        self.create_subscription(
            String, '/waypoint_status',
            self.waypoint_callback, 10)

        self.create_subscription(
            String, '/safety_status',
            self.safety_callback, 10)

        # --- Publisher ---
        self.state_pub = self.create_publisher(String, '/buggy_state', 10)

        # --- Status timer: publish state at 1 Hz ---
        self.create_timer(1.0, self.publish_state)

        self.get_logger().info('State machine started — state: IDLE')

    # ------------------------------------------------------------------
    def goal_callback(self, msg):
        """New goal received — transition to PLANNING."""
        if self.state in [self.IDLE, self.ARRIVED, self.ERROR]:
            self._transition(self.PLANNING)

    def planning_callback(self, msg):
        """Path planner status updates."""
        if msg.data.startswith('PLANNING:'):
            self._transition(self.NAVIGATING)
        elif msg.data.startswith('ERROR:'):
            self.get_logger().error(f'Planner error: {msg.data}')
            self._transition(self.ERROR)
        elif msg.data.startswith('Already at'):
            self._transition(self.ARRIVED)

    def waypoint_callback(self, msg):
        """Waypoint follower status updates."""
        if msg.data == 'MISSION_COMPLETE':
            self._transition(self.ARRIVED)
        elif msg.data.startswith('WAYPOINT_'):
            # Still navigating — log progress
            self.get_logger().info(f'Progress: {msg.data}')

    def safety_callback(self, msg):
        """Safety monitor stop/clear signals."""
        if msg.data == 'STOP' and self.state == self.NAVIGATING:
            self._transition(self.STOPPED)
        elif msg.data == 'CLEAR' and self.state == self.STOPPED:
            self._transition(self.NAVIGATING)

    # ------------------------------------------------------------------
    def _transition(self, new_state):
        """Perform a state transition with logging."""
        if new_state == self.state:
            return
        self.get_logger().info(
            f'State: {self.state} --> {new_state}')
        self.previous_state = self.state
        self.state = new_state
        self.publish_state()

    def publish_state(self):
        """Publish current state to /buggy_state."""
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
