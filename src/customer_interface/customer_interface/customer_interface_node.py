#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CustomerInterface(Node):
    def __init__(self):
        super().__init__('customer_interface')

        self.order_pub = self.create_publisher(String, '/orders', 10)
        self.cancel_pub = self.create_publisher(String, '/cancel_order', 10)

        self.get_logger().info("Customer Interface Started")

        self.run_cli()

    def run_cli(self):
        while rclpy.ok():
            print("\n--- Cafe Order System ---")
            print("1. Place Order")
            print("2. Cancel Order")
            choice = input("Select option: ")

            if choice == "1":
                table = input("Enter table (table1/table2/table3): ")
                msg = String()
                msg.data = table
                self.order_pub.publish(msg)
                print(f"Order placed for {table}")

            elif choice == "2":
                table = input("Enter table to cancel: ")
                msg = String()
                msg.data = table
                self.cancel_pub.publish(msg)
                print(f"Order cancelled for {table}")

            else:
                print("Invalid option")


def main():
    rclpy.init()
    node = CustomerInterface()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
