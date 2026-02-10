#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        self.spawn_cli = self.create_client(Spawn, '/spawn')
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')

        # Spawn cafe landmarks
        self.spawn_turtle('home', 5.5, 5.5)
        self.spawn_turtle('kitchen', 9.0, 9.0)
        self.spawn_turtle('table1', 2.0, 8.0)
        self.spawn_turtle('table2', 8.0, 2.0)
        self.spawn_turtle('table3', 2.0, 2.0)

        self.get_logger().info('Cafe layout spawned successfully')

    def spawn_turtle(self, name, x, y):
        req = Spawn.Request()
        req.name = name
        req.x = x
        req.y = y
        req.theta = 0.0
        self.spawn_cli.call_async(req)


def main():
    rclpy.init()
    node = TurtleController()
    rclpy.spin_once(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
