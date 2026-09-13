#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import (
    MultiThreadedExecutor,
)  # Allow multiple ROS2 callbacks to execute concurrently using multiple threads
from rclpy.callback_groups import (
    ReentrantCallbackGroup,
)  # Allow callbacks in this group to run concurrently,including while another callback from the same group is still executing
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
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
        self.callback_group = (
            ReentrantCallbackGroup()
        )  # Allow callbacks assigned to this group to run concurrently (needed because tha action callback blocks while navigation runs, which would otherwise block the robot's status timer)

        self.status = "IDLE"
        self.status_publisher_ = self.create_publisher(RobotStatus, "/robot/status", 10)
        self.timer_ = self.create_timer(
            0.5, self.publish_status, self.callback_group
        )  # share the reentrant callback group sot can continue running while the robot is executing a task
        self.register_client_ = self.create_client(
            RegisterRobot, "/fleet_manager/register_robot"
        )
        self.nav_client_ = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self.callback_group
        )  # namespaced under this robot, e.g. /robot_1/navigate_to_pose
        self.task_action_server = ActionServer(
            self,
            AssignTask,
            "assign_task",
            self.execute_assign_task,
            callback_group=self.callback_group,  # The action callback may take several seconds to complete.
            # Using the same reentrant callback group allows status publishing
            # to continue while the action is executing.
        )
        self.get_logger().info("Robot Node has been started")

    def publish_status(self):
        message = RobotStatus()
        message.robot_id = self.robot_id
        message.status = self.status
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
        dest_x = goal_handle.request.dest_x
        dest_y = goal_handle.request.dest_y

        self.get_logger().info(
            f"Received task {task_id}: {source}->{destination} ({dest_x:.2f}, {dest_y:.2f})"
        )
        self.status = "MOVING"
        feedback_msg = AssignTask.Feedback()
        feedback_msg.status = "MOVING"
        feedback_msg.progress = 0.0
        goal_handle.publish_feedback(feedback_msg)

        result = AssignTask.Result()

        if not self.nav_client_.wait_for_server(timeout_sec=5.0):
            self.status = "IDLE"
            goal_handle.abort()
            result.success = False
            result.message = f"Task {task_id} failed: navigate_to_pose action server unavailable"
            return result

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = "map"
        nav_goal.pose.pose.position.x = dest_x
        nav_goal.pose.pose.position.y = dest_y
        nav_goal.pose.pose.orientation.w = 1.0

        progress_state = {"initial_distance": None, "progress": 0.0}

        def nav_feedback_cb(feedback):
            remaining = feedback.feedback.distance_remaining
            if progress_state["initial_distance"] is None and remaining > 0.0:
                progress_state["initial_distance"] = remaining
            initial = progress_state["initial_distance"]
            if initial:
                progress_state["progress"] = max(
                    0.0, min(100.0, 100.0 * (1.0 - remaining / initial))
                )
            goal_handle.publish_feedback(
                AssignTask.Feedback(status="MOVING", progress=progress_state["progress"])
            )

        send_future = self.nav_client_.send_goal_async(
            nav_goal, feedback_callback=nav_feedback_cb
        )
        while not send_future.done():
            time.sleep(0.1)
        nav_goal_handle = send_future.result()

        if not nav_goal_handle.accepted:
            self.status = "IDLE"
            goal_handle.abort()
            result.success = False
            result.message = f"Task {task_id} failed: navigation goal rejected"
            return result

        result_future = nav_goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.2)

        nav_status = result_future.result().status
        self.status = "IDLE"

        if nav_status == GoalStatus.STATUS_SUCCEEDED:
            feedback_msg.status = "MOVING"
            feedback_msg.progress = 100.0
            goal_handle.publish_feedback(feedback_msg)
            goal_handle.succeed()
            result.success = True
            result.message = f"Task {task_id} completed successfully"
        else:
            goal_handle.abort()
            result.success = False
            result.message = f"Task {task_id} failed: navigation did not succeed (status={nav_status})"

        return result


def main(args=None):
    rclpy.init(args=args)
    node = Robot()
    node.register_robot()
    executor = MultiThreadedExecutor()  # the executor provides multiple threads
    executor.add_node(node)
    try:
        executor.spin()  # starts the executor and continuously processes the node's callbacks
    except KeyboardInterrupt:
        print(f"\nShutting down Robot")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
