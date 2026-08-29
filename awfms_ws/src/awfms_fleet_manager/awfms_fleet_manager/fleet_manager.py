#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from awfms_interfaces.msg import RobotStatus
from awfms_interfaces.srv import RegisterRobot, CreateTask, AssignTask
from functools import partial


class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager")
        self.robot_registry = {}  # hold all the registered robots
        self.task_registry = {}  # store all created warehouse tasks
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
        self.task_client_ = self.create_client(AssignTask, "assign_task")
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
        service_name = f"/{robot_id}/assign_task"
        task_client = self.create_client(
            AssignTask, service_name
        )  # create local client for the selected robot's assign_task service

        while not task_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for service")

        request = AssignTask.Request()

        request.task_id = task_id
        request.source = self.task_registry[task_id]["source"]
        request.destination = self.task_registry[task_id]["destination"]

        self.get_logger().info(f"Assigned {task_id} to {robot_id}")
        future = task_client.call_async(request)
        future.add_done_callback(
            partial(self.callback_assign_task, request=request, robot_id=robot_id)
        )
        return robot_id

    def callback_assign_task(self, future, request, robot_id):
        response = future.result()
        if response.success:
            self.task_registry[request.task_id]["assigned_robot"] = robot_id
            self.task_registry[request.task_id]["status"] = "ASSIGNED"

            self.robot_registry[request.robot_id]["status"] = "BUSY"
            self.robot_registry[request.robot_id]["available"] = False
            self.get_logger().info(
                f"Task assignment successful! message: {response.message}"
            )
        else:
            self.get_logger().info(
                f"Task assignment failed. message: {response.message}"
            )

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
                f"Type: {robot_info["type"]} | "
                f"Status: {robot_info["status"]} | "
                f"Available: {robot_info["available"]}\n"
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
