#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String 
from awfms_interfaces.srv import RegisterRobot

class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager") 
        self.robot_registry={} #hold all the registered robots
        self.status_publisher_ = self.create_publisher(String, "/fleet_manager/status",10)
        self.timer_ = self.create_timer(0.5, self.publish_status)
        self.robot_status_subscriber_ = self.create_subscription(String, "/robot/status",self.callback_robot_status, 10)
        self.register_service=self.create_service(RegisterRobot,"/fleet_manager/register_robot",self.register_robot_callback)
        self.get_logger().info("Fleet Manager Node has been started")

    def callback_robot_status(self,msg:String):
        self.get_logger().info(f"Received robot status: {msg.data}")

    def register_robot_callback(self,request:RegisterRobot.Request,response:RegisterRobot.Response):
        self.get_logger().info(f"Registration request received from: {request.robot_id}")
        if request.robot_id in self.robot_registry:
            response.success=False
            response.message=f"{request.robot_id} already registered"
            self.get_logger().warn(response.message)
        else:
            self.robot_registry[request.robot_id]={
                "type":request.robot_type
            }
            response.success=True
            response.message=f"{request.robot_id} registered successfully"
            self.get_logger().info(response.message)
        return response
    

    def publish_status(self):
        self.get_logger().info("Publishing message: Fleet manager is availiable")
        message = String()
        message.data="Fleet manager is available"
        self.status_publisher_.publish(message)


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
