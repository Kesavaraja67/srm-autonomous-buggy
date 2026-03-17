#!/usr/bin/env python3
"""
set_destination.py — Team Bravo interactive destination selector.
Run this in a separate terminal while the buggy system is running:
    ros2 run buggy_brain set_destination
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

VALID_DESTINATIONS = {
    'A': 'SRM_IST',
    'B': 'SRM_HOSP',
    'C': 'SRM_TEMPLE',
    'D': 'BUGGY_HUB',
}

MENU = (
    "\n"
    "========================================\n"
    "  SRM Autonomous Campus Buggy\n"
    "  Select Destination:\n"
    "    (A) SRM Institute (North)\n"
    "    (B) SRM Hospital (East)\n"
    "    (C) SRM Campus Temple (South)\n"
    "    (D) Buggy Hub (Base)\n"
    "  Type a letter and press Enter: "
)


def main(args=None):
    rclpy.init(args=args)
    node = Node('destination_cli')
    pub = node.create_publisher(String, '/destination_request', 10)

    # Wait briefly for publisher to connect
    import time
    time.sleep(1.0)

    print("\n[Destination CLI] Connected to buggy system.")
    print("[Destination CLI] Publishing to /destination_request topic.\n")

    try:
        while rclpy.ok():
            try:
                raw = input(MENU)
            except EOFError:
                break

            choice = raw.strip().upper()

            if choice == 'Q':
                print("[Destination CLI] Quitting.")
                break

            if choice not in VALID_DESTINATIONS:
                print(f"  Invalid choice '{raw}'. Enter A, B, C, or D (or Q to quit).")
                continue

            dest = VALID_DESTINATIONS[choice]
            msg = String()
            msg.data = choice
            pub.publish(msg)
            print(f"  [Destination CLI] Sent destination: {dest}")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
