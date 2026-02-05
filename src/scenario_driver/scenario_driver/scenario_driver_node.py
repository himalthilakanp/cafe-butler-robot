#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ScenarioDriver(Node):

    def __init__(self):
        super().__init__('scenario_driver')

        self.order_pub = self.create_publisher(String, '/orders', 10)
        self.cancel_pub = self.create_publisher(String, '/cancel_order', 10)

        self.scenario_index = 0
        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(1.0, self.run_scenarios)

        self.get_logger().info("Auto Scenario Driver started")

        self.status_sub = self.create_subscription(
            String,
            '/amr_status',
            self.status_callback,
            10
        )

        self.robot_idle = True
        
    def status_callback(self, msg):
        if msg.data.startswith("IDLE_AT_HOME"):
            self.robot_idle = True
        else:
            self.robot_idle = False



    def run_scenarios(self):
        if not self.robot_idle:
            return  # wait until robot is free

        if self.scenario_index == 0:
            self.get_logger().info("Scenario 1: Single order -> kitchen -> table -> home")
            self.publish_order("table1")

        elif self.scenario_index == 1:
            self.get_logger().info("Scenario 2: No confirmation -> timeout -> home")
            self.publish_order("table1")

        elif self.scenario_index == 2:
            self.get_logger().info("Scenario 3: No table confirmation -> kitchen -> home")
            self.publish_order("table1")

        elif self.scenario_index == 3:
            self.get_logger().info("Scenario 4: Cancel while going to table")
            self.publish_order("table1")
            self.schedule_cancel("table1", delay=2.0)

        elif self.scenario_index == 4:
            self.get_logger().info("Scenario 5: Multiple orders sequential delivery")
            self.publish_order("table1,table2,table3")

        elif self.scenario_index == 5:
            self.get_logger().info("Scenario 6: Skip table1, deliver table2 & table3")
            self.publish_order("table1,table2,table3")

        elif self.scenario_index == 6:
            self.get_logger().info("Scenario 7: Cancel table2, deliver others")
            self.publish_order("table1,table2,table3")
            self.schedule_cancel("table2", delay=6.0)

        else:
            self.get_logger().info("All scenarios completed")
            self.timer.cancel()
            return

        self.robot_idle = False
        self.scenario_index += 1


    def publish_order(self, tables):
        msg = String()
        msg.data = tables
        self.order_pub.publish(msg)
        self.get_logger().info(f"Published order: {tables}")

    def schedule_cancel(self, table, delay):
        cancel_timer = None

        def one_shot_cancel():
            nonlocal cancel_timer
            self.publish_cancel(table)
            cancel_timer.cancel()

        cancel_timer = self.create_timer(delay, one_shot_cancel)


    def publish_cancel(self, table):
        msg = String()
        msg.data = table
        self.cancel_pub.publish(msg)
        self.get_logger().info(f"Cancelled order for: {table}")

    def advance_scenario(self, wait_time):
        self.scenario_index += 1
        self.start_time = self.get_clock().now()
        self.get_logger().info(f"Waiting {wait_time}s before next scenario")


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioDriver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
