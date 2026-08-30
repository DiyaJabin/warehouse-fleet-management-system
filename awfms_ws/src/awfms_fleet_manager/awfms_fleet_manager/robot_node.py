#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from awfms_interfaces.msg import RobotStatus
from awfms_interfaces.srv import RegisterRobot
from awfms_interfaces.action import AssignTask


class Robot(Node):
    def __init__(self):
        super().__init__("robot")
        self.declare_parameter("robot_id", "")
        self.declare_parameter("robot_type", "")
        self.robot_id = self.get_parameter(
            "robot_id"
        ).value  # get the parameter value from the YAML file
        self.robot_type = self.get_parameter("robot_type").value
        self.status_publisher_ = self.create_publisher(RobotStatus, "/robot/status", 10)
        self.timer_ = self.create_timer(0.5, self.publish_status)
        self.register_client_ = self.create_client(
            RegisterRobot, "/fleet_manager/register_robot"
        )
        self.task_action_server = ActionServer(
            self, AssignTask, f"assign_task", self.execute_assign_task
        )
        self.get_logger().info("Robot Node has been started")

    def publish_status(self):
        message = RobotStatus()
        message.robot_id = self.robot_id
        message.status = "IDLE"
        self.status_publisher_.publish(message)

    def register_robot(self):
        while not self.register_client_.wait_for_service(
            timeout_sec=1.0
        ):  # wait for server to start, if not display the following
            self.get_logger().warn("Waiting for service....")

        request = RegisterRobot.Request()
        request.robot_id = self.robot_id
        request.robot_type = self.robot_type

        future = self.register_client_.call_async(
            request
        )  # asynchronous call used to avoid deadlock
        future.add_done_callback(self.callback_register_robot)

    def callback_register_robot(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info(f"Registration successfull: {response.message}")
        else:
            self.get_logger().info(f"Registration failed: {response.message}")

    def execute_assign_task(
        self, goal_handle
    ):  # goal_handle is the ROS2 handle for currently running action
        task_id = goal_handle.request.task_id
        source = goal_handle.request.source
        destination = goal_handle.request.destination

        self.get_logger().info(f"Received task {task_id}: {source}->{destination}")
        feedback_msg = AssignTask.Feedback()
        for progress in range(0, 101, 20):
            feedback_msg.status = "MOVING"
            feedback_msg.progress = float(progress)

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"{task_id} progress: {progress}")
            time.sleep(1)

        goal_handle.succeed()
        result = AssignTask.Result()
        result.success = True
        result.message = f"Task {task_id} completed successfully"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = Robot()
    node.register_robot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\nShutting down Robot")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
