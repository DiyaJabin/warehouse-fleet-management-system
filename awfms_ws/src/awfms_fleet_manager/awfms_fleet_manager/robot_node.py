#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from awfms_interfaces.srv import RegisterRobot

class Robot(Node):
    def __init__(self):
        super().__init__("robot_1")
        self.status_publisher_=self.create_publisher(String,"/robot/status",10)
        self.timer_ = self.create_timer(0.5,self.publish_status)
        self.register_client_=self.create_client(RegisterRobot,"/fleet_manager/register_robot")
        self.get_logger().info("Robot Node has been started")

    def publish_status(self):
        self.get_logger().info("Publishing message: Robot 1: IDLE")
        message=String()
        message.data="Robot 1: IDLE"
        self.status_publisher_.publish(message)

    def register_robot(self,robot_id:str,robot_type:str):
        while not self.register_client_.wait_for_service(timeout_sec=1.0): #wait for server to start, if not display the following
            self.get_logger().warn("Waiting for service....")

        request=RegisterRobot.Request()
        request.robot_id=robot_id
        request.robot_type=robot_type

        future=self.register_client_.call_async(request) #asynchronous call used to avoid deadlock
        future.add_done_callback(self.callback_register_robot)

    def callback_register_robot(self,future):
        response=future.result()
        if response.success:
            self.get_logger().info(f"Registration successfull: {response.message}")
        else:
            self.get_logger().info(f"Registration failed: {response.message}")




def main(args=None):
    rclpy.init(args=args)
    node=Robot()
    node.register_robot("robot_1","AGV")
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
