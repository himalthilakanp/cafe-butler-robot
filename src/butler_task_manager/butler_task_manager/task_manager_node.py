#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from collections import deque
import math

TABLE_DWELL_SEC = 5.0
HOME_RETURN_SEC = 4.0

HOME = (5.5, 5.5)
KITCHEN = (9.0, 9.0)

TABLES = {
    "table1": (2.0, 8.0),
    "table2": (8.0, 2.0),
    "table3": (2.0, 2.0),
}


class ButlerTaskManager(Node):

    def __init__(self):
        super().__init__("butler_task_manager")

        # Publishers
        self.status_pub = self.create_publisher(String, "/amr_status", 10)
        self.cmd_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Subscribers
        self.create_subscription(String, "/orders", self.order_callback, 10)
        self.create_subscription(String, "/cancel_order", self.cancel_callback, 10)
        self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        # FSM state
        self.current_status = "IDLE_AT_HOME"
        self.order_queue = deque()
        self.current_table = None
        self.current_pose = None

        # Cancel flags
        self.cancel_requested = False
        self.cancel_table = None

        # Timers
        self.create_timer(0.1, self.handle_movement)
        self.create_timer(2.0, self.publish_status)

        self.state_start_time = self.get_clock().now()
        self.get_logger().info("Butler Task Manager started")

    # ---------------- Callbacks ----------------

    def pose_callback(self, msg):
        self.current_pose = msg

    def order_callback(self, msg):
        self.get_logger().info(f"Order received: {msg.data}")

        tables = [t.strip().lower() for t in msg.data.split(",") if t.strip()]
        for table in tables:
            if table in TABLES:
                self.order_queue.append(table)
            else:
                self.get_logger().warn(f"Unknown table: {table}")

        if self.current_status == "IDLE_AT_HOME":
            self.current_status = "GOING_TO_KITCHEN"
            self.state_start_time = self.get_clock().now()

    def cancel_callback(self, msg):
        table = msg.data.strip().lower()
        self.get_logger().info(f"Cancellation received for: {table}")

        # Remove from pending queue
        self.order_queue = deque(t for t in self.order_queue if t != table)

        # Cancel current table if active
        if self.current_table == table:
            self.cancel_requested = True
            self.cancel_table = table

    def publish_status(self):
        msg = String()
        msg.data = f"{self.current_status} | Pending orders: {len(self.order_queue)}"
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    # ---------------- Motion ----------------

    def move_to(self, target):
        if self.current_pose is None:
            return

        dx = target[0] - self.current_pose.x
        dy = target[1] - self.current_pose.y
        distance = math.sqrt(dx * dx + dy * dy)

        cmd = Twist()

        if distance > 0.15:
            goal_theta = math.atan2(dy, dx)
            diff = goal_theta - self.current_pose.theta

            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi

            cmd.linear.x = min(2.0, distance)
            cmd.angular.z = 4.0 * diff
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # ---------------- FSM ----------------

    def handle_movement(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds / 1e9

        # Cancel while going to table → go back to kitchen
        if self.cancel_requested and self.current_status.startswith("GOING_TO_"):
            self.get_logger().info("Order cancelled while going to table, returning to kitchen")
            self.current_table = None
            self.current_status = "GOING_TO_KITCHEN"
            self.cancel_requested = False
            self.state_start_time = now
            return

        if self.current_status == "GOING_TO_KITCHEN":
            self.move_to(KITCHEN)
            if elapsed >= 5.0:
                self.current_status = "AT_KITCHEN"
                self.state_start_time = now

        elif self.current_status == "AT_KITCHEN":
            if self.order_queue:
                self.current_table = self.order_queue.popleft()
                self.current_status = f"GOING_TO_{self.current_table.upper()}"
            else:
                self.current_status = "GOING_TO_HOME"
            self.state_start_time = now

        elif self.current_status.startswith("GOING_TO_") and self.current_table:
            self.move_to(TABLES[self.current_table])
            if elapsed >= 4.0:
                self.current_status = f"AT_{self.current_table.upper()}"
                self.state_start_time = now

        elif self.current_status.startswith("AT_"):
            if elapsed >= TABLE_DWELL_SEC:
                if self.order_queue:
                    self.current_table = self.order_queue.popleft()
                    self.current_status = f"GOING_TO_{self.current_table.upper()}"
                else:
                    self.current_table = None
                    self.current_status = "GOING_TO_HOME"
                self.state_start_time = now

        elif self.current_status == "GOING_TO_HOME":
            self.move_to(HOME)
            if elapsed >= HOME_RETURN_SEC:
                self.current_status = "IDLE_AT_HOME"
                self.state_start_time = now


def main():
    rclpy.init()
    node = ButlerTaskManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
