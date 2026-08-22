#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String 

class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager") 
        self.publisher_ = self.create_publisher(String, "/fleet_manager/status",10)
        self.timer_ = self.create_timer(0.5, self.publish_status)
        self.get_logger().info("Fleet Manager Node has been started")

    def publish_status(self):
        self.get_logger().info("Publishing message: Fleet manager is availiable")
        message = String()
        message.data="Fleet manager is available"
        self.publisher_.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node=FleetManager()
    try:
        rclpy.spin(node) #keep the node running until shutdown
    except KeyboardInterrupt:
        print(f"\nShutting down fleet manager") #handle Ctrl+C gracefully
    finally:
        node.destroy_node() #destroy the node and release its ROS2  resources.
        if rclpy.ok(): #check whether ROS2 is still running before shutdown
            rclpy.shutdown()

if __name__=="__main__":
    main()
