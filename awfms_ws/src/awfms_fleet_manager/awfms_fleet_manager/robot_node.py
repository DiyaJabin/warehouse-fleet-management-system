#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class Robot(Node):
    def __init__(self):
        super().__init__("robot_1")
        self.publisher_=self.create_publisher(String,"/robot/status",10)
        self.timer_ = self.create_timer(0.5,self.publish_status)
        self.get_logger().info("Robot Node has been started")

    def publish_status(self):
        self.get_logger().info("Publishing message: Robot 1: IDLE")
        message=String()
        message.data="Robot 1: IDLE"
        self.publisher_.publish(message)

def main(args=None):
    rclpy.init(args=args)
    node=Robot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\nShutting down Robot")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()
