import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StateMachine(Node):
    """
    ROS 2 node that manages overall buggy behaviour state.

    States:
        WAITING_FOR_DESTINATION  — system ready, no goal yet
        PLANNING                 — path planner computing route
        NAVIGATING               — buggy driving toward destination
        EMERGENCY_STOP           — obstacle detected, halted
        RESUMING                 — obstacle cleared, resuming
        CROWD_DETECTED           — crowd in path, driver takeover
        MANUAL_CONTROL           — human driver in control
        DESTINATION_REACHED      — arrived at goal
        ERROR                    — something went wrong

    Subscribes : /goal_destination  (std_msgs/String)
                 /planning_status   (std_msgs/String)
                 /waypoint_status   (std_msgs/String)
                 /safety_status     (std_msgs/String)
                 /crowd_detected    (std_msgs/String)
    Publishes  : /buggy_state       (std_msgs/String)
                 /driver_alert      (std_msgs/String)
    """

    WAITING_FOR_DESTINATION = 'WAITING_FOR_DESTINATION'
    PLANNING                = 'PLANNING'
    NAVIGATING              = 'NAVIGATING'
    EMERGENCY_STOP          = 'EMERGENCY_STOP'
    RESUMING                = 'RESUMING'
    CROWD_DETECTED          = 'CROWD_DETECTED'
    MANUAL_CONTROL          = 'MANUAL_CONTROL'
    DESTINATION_REACHED     = 'DESTINATION_REACHED'
    ERROR                   = 'ERROR'

    def __init__(self):
        super().__init__('state_machine')

        self.state          = self.WAITING_FOR_DESTINATION
        self.previous_state = None
        self.current_goal   = None

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

        self.create_subscription(
            String, '/crowd_detected',
            self.crowd_callback, 10)

        self.create_subscription(
            String, '/manual_override',
            self.manual_callback, 10)

        # --- Publishers ---
        self.state_pub  = self.create_publisher(String, '/buggy_state',    10)
        self.alert_pub  = self.create_publisher(String, '/driver_alert',   10)

        # --- Status timer: publish state at 1 Hz ---
        self.create_timer(1.0, self.publish_state)

        self.get_logger().info(
            f'State machine started — state: {self.state}')

    # ------------------------------------------------------------------
    def goal_callback(self, msg):
        """New destination received."""
        if self.state in [
            self.WAITING_FOR_DESTINATION,
            self.DESTINATION_REACHED,
            self.ERROR
        ]:
            self.current_goal = msg.data.strip().upper()
            self.get_logger().info(f'New goal: {self.current_goal}')
            self._transition(self.PLANNING)

    def planning_callback(self, msg):
        """Path planner status updates."""
        if msg.data.startswith('PLANNING:'):
            self._transition(self.NAVIGATING)
        elif msg.data.startswith('ERROR:'):
            self.get_logger().error(f'Planner error: {msg.data}')
            self._publish_alert(f'PLANNING FAILED: {msg.data}')
            self._transition(self.ERROR)
        elif msg.data.startswith('Already at'):
            self._transition(self.DESTINATION_REACHED)

    def waypoint_callback(self, msg):
        """Waypoint follower status updates."""
        if msg.data == 'MISSION_COMPLETE':
            self.get_logger().info(
                f'Arrived at {self.current_goal}')
            self._publish_alert(
                f'ARRIVED: {self.current_goal}')
            self._transition(self.DESTINATION_REACHED)
        elif msg.data.startswith('WAYPOINT_'):
            self.get_logger().info(f'Progress: {msg.data}')

    def safety_callback(self, msg):
        """Obstacle safety signals."""
        if msg.data == 'STOP':
            if self.state == self.NAVIGATING:
                self._publish_alert('OBSTACLE DETECTED — emergency stop')
                self._transition(self.EMERGENCY_STOP)

        elif msg.data == 'CLEAR':
            if self.state == self.EMERGENCY_STOP:
                self._publish_alert('Obstacle cleared — resuming')
                self._transition(self.RESUMING)
                # Auto transition to NAVIGATING after brief resume
                self.create_timer(2.0, self._resume_navigation)

    def crowd_callback(self, msg):
        """Crowd detection signals."""
        if msg.data == 'CROWD_DETECTED':
            if self.state in [self.NAVIGATING, self.EMERGENCY_STOP]:
                self.get_logger().warn(
                    'Crowd detected — switching to manual control')
                self._publish_alert(
                    'CROWD DETECTED — driver takeover required')
                self._transition(self.CROWD_DETECTED)
                self.create_timer(1.0, self._enter_manual_control)

        elif msg.data == 'CROWD_CLEARED':
            if self.state == self.MANUAL_CONTROL:
                self.get_logger().info(
                    'Crowd cleared — returning to autonomous')
                self._publish_alert('Crowd cleared — resuming autonomous')
                self._transition(self.NAVIGATING)

    def manual_callback(self, msg):
        """Manual override signals."""
        if msg.data == 'ENGAGE':
            self._transition(self.MANUAL_CONTROL)
            self._publish_alert('Manual control engaged')
        elif msg.data == 'DISENGAGE':
            if self.state == self.MANUAL_CONTROL:
                self._transition(self.NAVIGATING)
                self._publish_alert('Autonomous control resumed')

    # ------------------------------------------------------------------
    def _resume_navigation(self):
        """Called 2 seconds after RESUMING to go back to NAVIGATING."""
        if self.state == self.RESUMING:
            self._transition(self.NAVIGATING)

    def _enter_manual_control(self):
        """Called 1 second after CROWD_DETECTED."""
        if self.state == self.CROWD_DETECTED:
            self._transition(self.MANUAL_CONTROL)

    def _transition(self, new_state):
        """Perform a state transition with logging."""
        if new_state == self.state:
            return
        self.get_logger().info(
            f'State: {self.state} --> {new_state}')
        self.previous_state = self.state
        self.state          = new_state
        self.publish_state()

    def publish_state(self):
        """Publish current state to /buggy_state."""
        msg      = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def _publish_alert(self, text):
        """Publish driver alert message."""
        msg      = String()
        msg.data = text
        self.alert_pub.publish(msg)
        self.get_logger().info(f'ALERT: {text}')


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
