#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.msg import String
from awfms_interfaces.msg import RobotStatus
from awfms_interfaces.srv import RegisterRobot, CreateTask
from awfms_interfaces.action import AssignTask
from functools import partial


class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager")
        self.robot_registry = {}  # hold all the registered robots
        self.task_registry = {}  # store all created warehouse tasks
        self.task_clients = {}
        self.status_publisher_ = self.create_publisher(
            String, "/fleet_manager/status", 10
        )
        self.timer_ = self.create_timer(0.5, self.publish_status)
        self.fleet_timer_ = self.create_timer(2.0, self.publish_fleet_status)
        self.offline_timer_ = self.create_timer(1.0, self.check_robot_timeouts)
        self.robot_status_subscriber_ = self.create_subscription(
            RobotStatus, "/robot/status", self.callback_robot_status, 10
        )
        self.register_service_ = self.create_service(
            RegisterRobot, "/fleet_manager/register_robot", self.register_robot_callback
        )
        self.task_service_ = self.create_service(
            CreateTask, "/fleet_manager/create_task", self.create_task_callback
        )
        self.get_logger().info("Fleet Manager Node has been started")

    def callback_robot_status(self, msg: RobotStatus):
        if msg.robot_id in self.robot_registry:
            self.robot_registry[msg.robot_id]["status"] = msg.status
            self.robot_registry[msg.robot_id]["available"] = msg.status == "IDLE"
            self.robot_registry[msg.robot_id]["last_seen"] = self.get_clock().now()
        else:
            self.get_logger().warning(
                f"Received status from unregistered robot: {msg.status}"
            )

    def register_robot_callback(
        self, request: RegisterRobot.Request, response: RegisterRobot.Response
    ):
        self.get_logger().info(
            f"Registration request received from: {request.robot_id}"
        )
        if request.robot_id in self.robot_registry:
            response.success = False
            response.message = f"{request.robot_id} already registered"
            self.get_logger().warn(response.message)
        else:
            self.robot_registry[request.robot_id] = {
                "type": request.robot_type,
                "status": "UNKNOWN",
                "available": False,
                "last_seen": self.get_clock().now(),  # returns current ROS2 time as a Time object
            }
            response.success = True
            response.message = f"{request.robot_id} registered successfully"
            self.get_logger().info(response.message)
        return response

    def find_available_robot(self):
        for robot_id, robot_info in self.robot_registry.items():
            if robot_info["available"]:
                return robot_id
        return None

    def assign_task(self, task_id):
        robot_id = self.find_available_robot()

        if robot_id is None:
            return None
        action_name = f"/{robot_id}/assign_task"
        self.task_clients[robot_id] = ActionClient(
            self, AssignTask, action_name
        )  # create local client for the selected robot

        task_client = self.task_clients[robot_id]

        while not task_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Waiting for action server")

        goal = AssignTask.Goal()

        goal.task_id = task_id
        goal.source = self.task_registry[task_id]["source"]
        goal.destination = self.task_registry[task_id]["destination"]

        self.get_logger().info(f"Assigned {task_id} to {robot_id}")
        future = task_client.send_goal_async(
            goal,
            feedback_callback=partial(
                self.callback_task_feedback, task_id=task_id, robot_id=robot_id
            ),
        )
        future.add_done_callback(
            partial(self.callback_assign_task, task_id=task_id, robot_id=robot_id)
        )
        return robot_id

    def callback_assign_task(self, future, task_id, robot_id):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.task_registry[task_id]["assigned_robot"] = robot_id
            self.task_registry[task_id]["status"] = "ASSIGNED"

            self.robot_registry[robot_id]["status"] = "BUSY"
            self.robot_registry[robot_id]["available"] = False
            self.get_logger().info(f"Task {task_id} accepted by {robot_id}")

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                partial(self.callback_task_result, task_id=task_id, robot_id=robot_id)
            )
        else:
            self.get_logger().info(f"Task {task_id} was rejected by {robot_id}")
            return

    def callback_task_feedback(self, feedback_msg, task_id, robot_id):
        feedback = feedback_msg.feedback

        self.task_registry[task_id]["status"] = feedback.status
        self.task_registry[task_id]["progress"] = feedback.progress

        self.get_logger().info(
            f"{task_id} | {robot_id} | " f"{feedback.status} | {feedback.progress}%"
        )

    def callback_task_result(self, future, task_id, robot_id):

        result = future.result().result

        if result.success:

            self.task_registry[task_id]["status"] = "COMPLETED"
            self.task_registry[task_id]["progress"] = 100.0

            self.robot_registry[robot_id]["status"] = "IDLE"
            self.robot_registry[robot_id]["available"] = True

            self.get_logger().info(
                f"Task {task_id} completed successfully by {robot_id}"
            )

        else:

            self.task_registry[task_id]["status"] = "FAILED"

            self.robot_registry[robot_id]["status"] = "IDLE"
            self.robot_registry[robot_id]["available"] = True

            self.get_logger().info(f"Task {task_id} failed: {result.message}")

    def create_task_callback(
        self, request: CreateTask.Request, response: CreateTask.Response
    ):
        self.get_logger().info(f"Task creation request received: {request.task_id}")
        if request.task_id in self.task_registry:
            response.success = False
            response.message = f"{request.task_id} already exists"
            self.get_logger().warn(response.message)
        else:
            self.task_registry[request.task_id] = {
                "source": request.source,
                "destination": request.destination,
                "priority": request.priority,
                "status": "PENDING",
                "assigned_robot": None,
            }

            robot_id = self.assign_task(request.task_id)
            if robot_id is not None:
                response.success = True
                response.message = f"{request.task_id} assigned to {robot_id} "

            else:
                response.success = True
                response.message = (
                    f"{request.task_id} created but no robot is available"
                )
        return response

    def publish_status(self):
        self.get_logger().info("Publishing message: Fleet manager is available")
        message = String()
        message.data = "Fleet manager is available"
        self.status_publisher_.publish(message)

    def publish_fleet_status(self):
        self.get_logger().info("-----Fleet-Manager-----")
        for robot_id, robot_info in self.robot_registry.items():
            self.get_logger().info(
                f"{robot_id} | "
                f"Type: {robot_info['type']} | "
                f"Status: {robot_info['status']} | "
                f"Available: {robot_info['available']}\n"
            )

    def check_robot_timeouts(self):
        current_time = self.get_clock().now()
        for robot_id, robot_info in self.robot_registry.items():
            time_since_last_seen = (
                current_time - robot_info["last_seen"]
            ).nanoseconds / 1e9  # convert the nanoseconds to seconds (1e9=1*10^9)
            if time_since_last_seen > 2.0:
                if robot_info["status"] != "OFFLINE":
                    robot_info["status"] = "OFFLINE"
                    robot_info["available"] = False
                    self.get_logger().warn(f"{robot_id} has gone OFFLINE")


def main(args=None):
    rclpy.init(args=args)
    node = FleetManager()
    try:
        rclpy.spin(node)  # keep the node running until shutdown
    except KeyboardInterrupt:
        print(f"\nShutting down fleet manager")  # handle Ctrl+C gracefully
    finally:
        node.destroy_node()  # destroy the node and release its ROS2  resources.
        if rclpy.ok():  # check whether ROS2 is still running before shutdown
            rclpy.shutdown()


if __name__ == "__main__":
    main()
