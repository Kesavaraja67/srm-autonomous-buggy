#!/usr/bin/env python3
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

STATE_WAITING    = 'WAITING_FOR_DESTINATION'
STATE_NAVIGATING = 'NAVIGATING'
STATE_ESTOP      = 'EMERGENCY_STOP'
STATE_RESUMING   = 'RESUMING'
STATE_CROWD      = 'CROWD_DETECTED'
STATE_MANUAL     = 'MANUAL_CONTROL'
STATE_ARRIVED    = 'DESTINATION_REACHED'

CLEAR_NEEDED      = 5
DETECT_NEEDED     = 3
RESUMING_DELAY_S  = 0.5
CROWD_COUNTDOWN_S = 3
FSM_RATE_HZ       = 10


class StateMachineNode(Node):

    def __init__(self):
        super().__init__('state_machine_node')
        self._state_pub = self.create_publisher(String, '/buggy_state', 10)
        self.create_subscription(Bool,   '/obstacle_detected',  self._obstacle_cb,   10)
        self.create_subscription(Bool,   '/ultrasonic_alert',   self._ultrasonic_cb, 10)
        self.create_subscription(Bool,   '/crowd_detected',     self._crowd_cb,      10)
        self.create_subscription(String, '/navigation_command', self._nav_cmd_cb,    10)
        self.create_subscription(Bool,   '/manual_override',    self._manual_cb,     10)

        self._obstacle_detected  = False
        self._ultrasonic_alert   = False
        self._crowd_detected     = False
        self._manual_cleared     = False
        self._consecutive_detections = 0
        self._consecutive_clears     = 0
        self._resuming_start     = None
        self._countdown_running  = False
        self._current_goal       = None
        self._state = STATE_WAITING

        self.create_timer(1.0 / FSM_RATE_HZ, self._fsm_tick)
        self.get_logger().info('StateMachineNode started. State: WAITING_FOR_DESTINATION')

    def _obstacle_cb(self, msg):
        if msg.data:
            self._consecutive_detections += 1
            self._consecutive_clears = 0
        else:
            self._consecutive_clears += 1
            self._consecutive_detections = 0
        self._obstacle_detected = msg.data

    def _ultrasonic_cb(self, msg):
        self._ultrasonic_alert = msg.data

    def _crowd_cb(self, msg):
        self._crowd_detected = msg.data

    def _nav_cmd_cb(self, msg):
        if msg.data.startswith('START:'):
            self._current_goal = msg.data.split(':')[1]
            if self._state in (STATE_WAITING, STATE_ARRIVED):
                self._transition(STATE_NAVIGATING)
        elif msg.data == 'DESTINATION_REACHED':
            if self._state in (STATE_NAVIGATING, STATE_RESUMING):
                self._transition(STATE_ARRIVED)

    def _manual_cb(self, msg):
        if not msg.data:
            self._manual_cleared = True

    def _fsm_tick(self):
        s = self._state
        if s == STATE_WAITING:
            pass
        elif s == STATE_NAVIGATING:
            if self._ultrasonic_alert:
                self._transition(STATE_ESTOP)
            elif self._consecutive_detections >= DETECT_NEEDED:
                self._transition(STATE_ESTOP)
            elif self._crowd_detected and not self._countdown_running:
                self._transition(STATE_CROWD)
        elif s == STATE_ESTOP:
            if not self._ultrasonic_alert and self._consecutive_clears >= CLEAR_NEEDED:
                self._transition(STATE_RESUMING)
        elif s == STATE_RESUMING:
            if self._resuming_start is None:
                self._resuming_start = time.monotonic()
            elif time.monotonic() - self._resuming_start >= RESUMING_DELAY_S:
                self._resuming_start = None
                self._transition(STATE_NAVIGATING)
        elif s == STATE_CROWD:
            pass
        elif s == STATE_MANUAL:
            if self._manual_cleared:
                self._manual_cleared = False
                self._transition(STATE_NAVIGATING)
        elif s == STATE_ARRIVED:
            pass
        self._publish_state()

    def _transition(self, new_state):
        if new_state == self._state:
            return
        self.get_logger().info(f'[STATE] {self._state} -> {new_state}')
        print(f'\n[STATE MACHINE] {self._state} -> {new_state}')
        self._state = new_state
        if new_state == STATE_ESTOP:
            self._consecutive_clears = 0
        elif new_state == STATE_CROWD and not self._countdown_running:
            threading.Thread(target=self._crowd_countdown, daemon=True).start()
        elif new_state == STATE_ARRIVED:
            goal = self._current_goal or '?'
            print(f'[NAVIGATION] Arrived at {goal}.')
            self._publish_state()
            self._transition(STATE_WAITING)

    def _crowd_countdown(self):
        self._countdown_running = True
        print('\n[STATE MACHINE] CROWD DETECTED - Requesting driver takeover')
        for i in range(CROWD_COUNTDOWN_S, 0, -1):
            print(f'[SAFETY] Handing control to driver in {i}...')
            time.sleep(1.0)
        print('[SAFETY] MANUAL_CONTROL active. Awaiting driver clearance.')
        self._countdown_running = False
        self._transition(STATE_MANUAL)

    def _publish_state(self):
        msg = String()
        msg.data = self._state
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
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
