#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager") 
        self.get_logger().info("Fleet Manager Node has been started")


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
