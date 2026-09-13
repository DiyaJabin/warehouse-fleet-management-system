#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.msg import String
from awfms_interfaces.msg import RobotStatus
from awfms_interfaces.srv import RegisterRobot, CreateTask, ReserveZone, ReleaseZone
from awfms_interfaces.action import AssignTask
from functools import partial

# Named locations in the warehouse (map frame), matching the pickup_zone/
# dropoff_zone markers placed in warehouse.sdf.
LOCATIONS = {
    "pickup": (-8.3, 0.0),
    "dropoff": (8.3, 0.0),
}

# Shared lane segments a route may need to cross, positioned over the
# central aisle between the two shelf rows (y=3.0 / y=-3.0) where every
# robot's path currently overlaps. (zone_id, x, y, radius)
ZONES = [
    ("corridor_west", -5.0, 0.0, 2.0),
    ("corridor_center", 0.0, 0.0, 2.5),
    ("corridor_east", 5.0, 0.0, 2.0),
]

ASSIGN_RETRY = "RETRY"
ASSIGN_INVALID = "INVALID"


def _dist_point_to_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return ((px - ax) ** 2 + (py - ay) ** 2) ** 0.5
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    cx, cy = ax + t * dx, ay + t * dy
    return ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5


class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager")
        self.robot_registry = {}  # hold all the registered robots
        self.task_registry = {}  # store all created warehouse tasks
        self.task_clients = {}
        self.pending_tasks = []  # task_ids waiting for a robot or a free lane
        self.zone_locks = {zone_id: None for zone_id, _, _, _ in ZONES}
        self.status_publisher_ = self.create_publisher(
            String, "/fleet_manager/status", 10
        )
        self.timer_ = self.create_timer(0.5, self.publish_status)
        self.fleet_timer_ = self.create_timer(2.0, self.publish_fleet_status)
        self.offline_timer_ = self.create_timer(1.0, self.check_robot_timeouts)
        self.pending_timer_ = self.create_timer(1.0, self.process_pending_tasks)
        self.robot_status_subscriber_ = self.create_subscription(
            RobotStatus, "/robot/status", self.callback_robot_status, 10
        )
        self.register_service_ = self.create_service(
            RegisterRobot, "/fleet_manager/register_robot", self.register_robot_callback
        )
        self.task_service_ = self.create_service(
            CreateTask, "/fleet_manager/create_task", self.create_task_callback
        )
        self.reserve_zone_service_ = self.create_service(
            ReserveZone, "/fleet_manager/reserve_zone", self.reserve_zone_callback
        )
        self.release_zone_service_ = self.create_service(
            ReleaseZone, "/fleet_manager/release_zone", self.release_zone_callback
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

    def zones_for_route(self, start, end):
        if start is None:
            return [zone_id for zone_id, _, _, _ in ZONES]
        ax, ay = start
        bx, by = end
        needed = []
        for zone_id, zx, zy, zr in ZONES:
            if _dist_point_to_segment(zx, zy, ax, ay, bx, by) <= zr:
                needed.append(zone_id)
        return needed

    def _try_reserve(self, robot_id, zone_id):
        holder = self.zone_locks.get(zone_id)
        if holder is None or holder == robot_id:
            self.zone_locks[zone_id] = robot_id
            return True
        return False

    def try_reserve_zones(self, robot_id, zone_ids):
        reserved = []
        for zone_id in zone_ids:
            if self._try_reserve(robot_id, zone_id):
                reserved.append(zone_id)
            else:
                for zid in reserved:
                    self.zone_locks[zid] = None
                return False
        return True

    def release_zones(self, robot_id, zone_ids):
        for zone_id in zone_ids:
            if self.zone_locks.get(zone_id) == robot_id:
                self.zone_locks[zone_id] = None

    def reserve_zone_callback(
        self, request: ReserveZone.Request, response: ReserveZone.Response
    ):
        if request.zone_id not in self.zone_locks:
            response.granted = False
            response.message = f"Unknown zone: {request.zone_id}"
            return response
        response.granted = self._try_reserve(request.robot_id, request.zone_id)
        if response.granted:
            response.message = f"{request.zone_id} granted to {request.robot_id}"
        else:
            response.message = (
                f"{request.zone_id} held by {self.zone_locks[request.zone_id]}"
            )
        return response

    def release_zone_callback(
        self, request: ReleaseZone.Request, response: ReleaseZone.Response
    ):
        if request.zone_id not in self.zone_locks:
            response.success = False
            response.message = f"Unknown zone: {request.zone_id}"
            return response
        self.release_zones(request.robot_id, [request.zone_id])
        response.success = True
        response.message = f"{request.zone_id} released by {request.robot_id}"
        return response

    def try_assign(self, task_id):
        task = self.task_registry[task_id]
        if task["destination"] not in LOCATIONS:
            self.get_logger().warn(
                f"{task_id}: unknown destination '{task['destination']}'"
            )
            return ASSIGN_INVALID

        robot_id = self.find_available_robot()
        if robot_id is None:
            return ASSIGN_RETRY

        start = LOCATIONS.get(task["source"])
        dest = LOCATIONS[task["destination"]]
        zone_ids = self.zones_for_route(start, dest)
        if not self.try_reserve_zones(robot_id, zone_ids):
            return ASSIGN_RETRY

        task["zones"] = zone_ids
        task["assigned_robot"] = robot_id
        task["status"] = "ASSIGNED"
        self.robot_registry[robot_id]["status"] = "BUSY"
        self.robot_registry[robot_id]["available"] = False
        self.dispatch_task(task_id, robot_id, dest)
        return robot_id

    def dispatch_task(self, task_id, robot_id, dest):
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
        goal.dest_x = dest[0]
        goal.dest_y = dest[1]

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

    def callback_assign_task(self, future, task_id, robot_id):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info(f"Task {task_id} accepted by {robot_id}")

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                partial(self.callback_task_result, task_id=task_id, robot_id=robot_id)
            )
        else:
            self.get_logger().info(f"Task {task_id} was rejected by {robot_id}")
            self.release_zones(robot_id, self.task_registry[task_id].get("zones", []))
            self.task_registry[task_id]["status"] = "FAILED"
            self.robot_registry[robot_id]["status"] = "IDLE"
            self.robot_registry[robot_id]["available"] = True

    def callback_task_feedback(self, feedback_msg, task_id, robot_id):
        feedback = feedback_msg.feedback

        self.task_registry[task_id]["status"] = feedback.status
        self.task_registry[task_id]["progress"] = feedback.progress

        self.get_logger().info(
            f"{task_id} | {robot_id} | " f"{feedback.status} | {feedback.progress}%"
        )

    def callback_task_result(self, future, task_id, robot_id):

        result = future.result().result
        self.release_zones(robot_id, self.task_registry[task_id].get("zones", []))

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
            return response

        self.task_registry[request.task_id] = {
            "source": request.source,
            "destination": request.destination,
            "priority": request.priority,
            "status": "PENDING",
            "assigned_robot": None,
        }

        result = self.try_assign(request.task_id)
        if result == ASSIGN_INVALID:
            response.success = False
            response.message = (
                f"{request.task_id}: unknown destination '{request.destination}'"
            )
            del self.task_registry[request.task_id]
        elif result == ASSIGN_RETRY:
            self.pending_tasks.append(request.task_id)
            response.success = True
            response.message = (
                f"{request.task_id} created and queued "
                "(waiting for a free robot or lane)"
            )
        else:
            response.success = True
            response.message = f"{request.task_id} assigned to {result} "
        return response

    def process_pending_tasks(self):
        still_pending = []
        for task_id in self.pending_tasks:
            result = self.try_assign(task_id)
            if result == ASSIGN_RETRY:
                still_pending.append(task_id)
            elif result == ASSIGN_INVALID:
                self.task_registry[task_id]["status"] = "FAILED"
        self.pending_tasks = still_pending

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
