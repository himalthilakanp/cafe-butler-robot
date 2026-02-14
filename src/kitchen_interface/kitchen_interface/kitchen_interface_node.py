#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading


class KitchenInterface(Node):

    def __init__(self):
        super().__init__('kitchen_interface')

        self.create_subscription(
            String,
            "/amr_status",
            self.status_callback,
            10
        )

        self.confirm_pub = self.create_publisher(
            String,
            "/kitchen_confirm",
            10
        )

        self.waiting_for_confirm = False

        self.get_logger().info("🍳 Kitchen Interface Started")

    # ------------------------------------------

    def status_callback(self, msg):

        # Only trigger when robot JUST reaches kitchen
        if msg.data.startswith("AT_KITCHEN"):

            if not self.waiting_for_confirm:

                self.waiting_for_confirm = True

                print("\n🍽 Robot reached kitchen.")
                print("Type 'ready' and press Enter to confirm.")

                threading.Thread(
                    target=self.wait_for_input,
                    daemon=True
                ).start()

    # ------------------------------------------

    def wait_for_input(self):

        user_input = input()

        if user_input.strip().lower() == "ready":

            msg = String()
            msg.data = "ready"
            self.confirm_pub.publish(msg)

            print("✅ Kitchen confirmed")

        else:
            print("❌ Invalid input")

        # Always reset
        self.waiting_for_confirm = False


# ------------------------------------------

def main():
    rclpy.init()
    node = KitchenInterface()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
