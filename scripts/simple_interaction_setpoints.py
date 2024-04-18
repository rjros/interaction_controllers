#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import interaction_controllers.msg


def main(args=None):
    rclpy.init(args=args)
    node = Node('rectangle_setpoint')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
