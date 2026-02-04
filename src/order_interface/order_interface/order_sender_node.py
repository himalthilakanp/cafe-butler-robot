#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class ordersender(Node):
    def __init__(self):
        super().__init__("order_sender")
        self.publisher_ = self.create_publisher(String,"/orders",20)
        self.timer = self.create_timer(2.0,self.sendorder)
        self.sent = False
        self.get_logger().info("Order Sender Started")


    def sendorder(self):
        if self.sent:
            return
        
        msg=String()
        msg.data = "table1,table2,table3"
        self.publisher_.publish(msg)
        self.get_logger().info(f"sent order for tables{msg.data}")
        self.sent = True
        



def main(args=None):
    rclpy.init(args=args)
    node = ordersender()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()
