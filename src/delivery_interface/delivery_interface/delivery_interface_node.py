#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading


class DeliveryInterface(Node):

    def __init__(self):
        super().__init__('delivery_interface')

        self.create_subscription(
            String,
            "/amr_status",
            self.status_callback,
            10
        )

        self.confirm_pub = self.create_publisher(
            String,
            "/customer_confirm",
            10
        )

        self.waiting_for_confirm = False

        self.get_logger().info("📦 Delivery Interface Started")


    def status_callback(self, msg):

        if msg.data.startswith("AT_TABLE:"):

            if not self.waiting_for_confirm:

                self.waiting_for_confirm = True

                table_name = msg.data.split(":")[1].split("|")[0].strip()

                print(f"\n🍽 Robot reached {table_name}.")
                print("Type 'received' and press Enter to confirm delivery.")

                threading.Thread(
                    target=self.wait_for_input,
                    daemon=True
                ).start()


    def wait_for_input(self):

        user_input = input()

        if user_input.strip().lower() == "received":

            msg = String()
            msg.data = "received"
            self.confirm_pub.publish(msg)

            print("✅ Delivery confirmed")

        else:
            print("❌ Invalid input")

        self.waiting_for_confirm = False


def main():
    rclpy.init()
    node = DeliveryInterface()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
