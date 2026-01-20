#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleTwistPublisher(Node):
    def __init__(self):
        super().__init__('simple_twist_publisher')

        self.publisher = self.create_publisher(
            Twist,
            '/adaptive_integral_terminal_sliding_mode_controller/reference',
            10
        )

        # =========================
        # SELECT MOTION HERE
        # =========================
        self.motion = "down"
        # Options:
        # "forward"
        # "backward"
        # "left"
        # "right"
        # "up"
        # "down"
        # "hover"
        # =========================

        self.speed = 0.3  # m/s

        self.timer = self.create_timer(0.1, self.publish_twist)  # 10 Hz
        self.get_logger().info(f"Publishing Twist: {self.motion}")

    def publish_twist(self):
        msg = Twist()

        if self.motion == "forward":
            msg.linear.x = self.speed
        elif self.motion == "backward":
            msg.linear.x = -self.speed
        elif self.motion == "left":
            msg.linear.y = self.speed
        elif self.motion == "right":
            msg.linear.y = -self.speed
        elif self.motion == "up":
            msg.linear.z = self.speed
        elif self.motion == "down":
            msg.linear.z = -self.speed
        elif self.motion == "hover":
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0

        # angular always zero
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = SimpleTwistPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
