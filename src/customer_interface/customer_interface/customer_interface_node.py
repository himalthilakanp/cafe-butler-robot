#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


VALID_TABLES = ["table1", "table2", "table3"]


class CustomerInterface(Node):
    def __init__(self):
        super().__init__('customer_interface')

        #publisher
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

            # ---------- PLACE ORDER ----------
            if choice == "1":
                table_input = input(
                    "Enter table(s) (table1,table2,table3): "
                )

                tables = [t.strip() for t in table_input.split(",") if t.strip()]
                invalid_tables = [t for t in tables if t not in VALID_TABLES]

                if not tables:
                    print("❌ No table entered")
                    continue

                if invalid_tables:
                    print(f"❌ Invalid table(s): {invalid_tables}")
                    print("✅ Valid options: table1, table2, table3")
                    continue

                msg = String()
                msg.data = ",".join(tables)
                self.order_pub.publish(msg)

                print(f"✅ Order placed for: {tables}")

            # ---------- CANCEL ORDER ----------
            elif choice == "2":
                table = input("Enter table to cancel (table1/table2/table3): ").strip()

                if table not in VALID_TABLES:
                    print(f"❌ Invalid table: {table}")
                    print("✅ Valid options: table1, table2, table3")
                    continue

                msg = String()
                msg.data = table
                self.cancel_pub.publish(msg)

                print(f"✅ Order cancelled for {table}")

            else:
                print("❌ Invalid option (choose 1 or 2)")


def main():
    rclpy.init()
    node = CustomerInterface()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
