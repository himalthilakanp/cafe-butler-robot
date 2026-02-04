#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from collections import deque


class ButlerTaskManager(Node):
    def __init__(self):
        super().__init__("butler_task_manager")
        
        # Publisher
        self.status_publisher = self.create_publisher(
            String,
            '/amr_status',
            10
        )

        # Subscriber
        self.order_subscriber = self.create_subscription(
            String,
            '/orders',
            self.order_callback,
            10
        )

        # Internal state
        self.current_status = "IDLE_AT_HOME"
        self.order_queue = deque()

        # Timer to publish status
        self.timer = self.create_timer(2.0, self.publish_status)
        
        # Movement simulation timer
        self.movement_timer = self.create_timer(1.0, self.handle_movement)

        self.state_start_time = self.get_clock().now()

        self.get_logger().info("Butler Task Manager started and waiting for orders")

        self.current_table = None



    def order_callback(self, msg: String):
        self.get_logger().info(f"Order received: {msg.data}")

        tables = [t.strip() for t in msg.data.split(',') if t.strip()]

        if not tables:
            self.get_logger().warn("Received empty order")
            return

        for table in tables:
            self.order_queue.append(table)

        # Change state only if idle
        if self.current_status == "IDLE_AT_HOME":
            self.current_status = "GOING_TO_KITCHEN"
            self.state_start_time = self.get_clock().now()
            self.get_logger().info("State changed to GOING_TO_KITCHEN")


    def publish_status(self):
        msg = String()
        msg.data = f"{self.current_status} | Pending orders: {len(self.order_queue)}"
        self.status_publisher.publish(msg)
        self.get_logger().info(msg.data)
    def handle_movement(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds / 1e9

        # Going to kitchen
        if self.current_status == "GOING_TO_KITCHEN":
            if elapsed >= 5.0:
                self.current_status = "AT_KITCHEN"
                self.state_start_time = now
                self.get_logger().info("Reached kitchen")

        # At kitchen: decide next table
        elif self.current_status == "AT_KITCHEN":
            if self.order_queue:
                self.current_table = self.order_queue.popleft()
                self.current_status = f"GOING_TO_{self.current_table.upper()}"
                self.state_start_time = now
                self.get_logger().info(f"Heading to {self.current_table}")

        # Going to table
        elif self.current_status.startswith("GOING_TO_"):
            if elapsed >= 4.0 and self.current_table is not None:
                self.current_status = f"AT_{self.current_table.upper()}"
                self.state_start_time = now
                self.get_logger().info(f"Reached {self.current_table}")


def main(args=None):
    rclpy.init(args=args)
    node= ButlerTaskManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
