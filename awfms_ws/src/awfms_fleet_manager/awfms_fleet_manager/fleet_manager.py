#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.msg import String
from awfms_interfaces.msg import RobotStatus
from awfms_interfaces.srv import RegisterRobot, CreateTask, ReserveZone, ReleaseZone
from awfms_interfaces.action import AssignTask
from functools import partial
from awfms_fleet_manager.fleet_db import FleetDB, DEFAULT_DB_PATH

# Named locations in the warehouse (map frame). pickup/dropoff match the
# pickup_zone/dropoff_zone markers in warehouse.sdf; the bays sit in the
# north/south aisles behind the two shelf rows (shelves at y=+-3.0).
LOCATIONS = {
    "pickup": (-8.3, 0.0),
    "dropoff": (8.3, 0.0),
    "bay_nw": (-7.5, 5.0),
    "bay_ne": (7.5, 5.0),
    "bay_sw": (-7.5, -5.0),
    "bay_se": (7.5, -5.0),
    # Deliberately inside shelf_2: Nav2 cannot reach it, which exercises the
    # task-failure path (task FAILED, zones released, robot freed).
    "fault_test": (0.0, 3.0),
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
TERMINAL_STATES = ("COMPLETED", "FAILED")


def _dist_point_to_segment(px, py, ax, ay, bx, by): #distance between zone center to the robot's route
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
        self.declare_parameter("db_path", DEFAULT_DB_PATH)
        self.db = FleetDB(self.get_parameter("db_path").value)
        self.db.set_locations(LOCATIONS)
        self.robot_registry = {}  # hold all the registered robots
        self.task_registry = {}  # store all created warehouse tasks
        self.task_clients = {}
        self.pending_tasks = []  # task_ids waiting for a robot or a free lane
        self.zone_locks = {zone_id: None for zone_id, _, _, _ in ZONES} #Take only the first value, don't care about the other values
        self.status_publisher_ = self.create_publisher(
            String, "/fleet_manager/status", 10
        )
        self.timer_ = self.create_timer(0.5, self.publish_status)
        self.fleet_timer_ = self.create_timer(2.0, self.publish_fleet_status)
        self.offline_timer_ = self.create_timer(1.0, self.check_robot_timeouts)
        self.pending_timer_ = self.create_timer(1.0, self.process_pending_tasks)
        self.db_timer_ = self.create_timer(1.0, self.sync_database)
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
        self.db.log_event("SYSTEM", f"Fleet manager started (run {self.db.run_id})")
        self.get_logger().info(f"Fleet Manager Node has been started (db: {self.get_parameter('db_path').value})")

    def log_event(self, category, message, robot_id=None, task_id=None, zone_id=None):
        self.get_logger().info(f"[{category}] {message}")
        self.db.log_event(category, message, robot_id=robot_id, task_id=task_id, zone_id=zone_id)

    def callback_robot_status(self, msg: RobotStatus):
        if msg.robot_id in self.robot_registry:
            robot = self.robot_registry[msg.robot_id]
            if robot["status"] == "OFFLINE":
                self.log_event("ROBOT", f"{msg.robot_id} is back online", robot_id=msg.robot_id)
            robot["status"] = msg.status
            # A robot stays unavailable while it owns a task, even if a stale IDLE
            # heartbeat arrives between dispatch and the robot starting the goal.
            robot["available"] = msg.status == "IDLE" and robot["current_task"] is None
            robot["x"], robot["y"] = msg.x, msg.y
            robot["last_seen"] = self.get_clock().now()
            robot["last_seen_wall"] = time.time()
        else:
            self.get_logger().warning(
                f"Received status from unregistered robot: {msg.robot_id}"
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
                "current_task": None,
                "x": math.nan,
                "y": math.nan,
                "last_seen": self.get_clock().now(),  # returns current ROS2 time as a Time object
                "last_seen_wall": time.time(),
            }
            response.success = True
            response.message = f"{request.robot_id} registered successfully"
            self.log_event("ROBOT", response.message, robot_id=request.robot_id)
        return response

    def available_robots(self, near=None):
        """Idle robots, nearest to `near` first (straight-line); unknown poses and ties by id."""
        def key(rid):
            robot = self.robot_registry[rid]
            if near is None or math.isnan(robot["x"]):
                return (math.inf, rid)
            return (math.hypot(robot["x"] - near[0], robot["y"] - near[1]), rid)
        return sorted((rid for rid, info in self.robot_registry.items() if info["available"]), key=key)

    def zones_for_route(self, waypoints):
        if any(p is None for p in waypoints):
            return [zone_id for zone_id, _, _, _ in ZONES] #Assume all zones might be needed if a waypoint is not known
        needed = [] #list of zones needed for the route
        segments = list(zip(waypoints, waypoints[1:]))
        for zone_id, zx, zy, zr in ZONES:
            #If any leg of the route comes within the zone's radius, that zone is required.
            if any(_dist_point_to_segment(zx, zy, *a, *b) <= zr for a, b in segments):
                needed.append(zone_id)
        return needed

    def _try_reserve(self, robot_id, zone_id):
        holder = self.zone_locks.get(zone_id)
        if holder is None or holder == robot_id:
            self.zone_locks[zone_id] = robot_id
            return True
        return False

    def try_reserve_zones(self, robot_id, zone_ids):
        """Reserve every zone or none of them. Returns (granted, blocking_zone)."""
        reserved = []
        for zone_id in zone_ids:
            if self._try_reserve(robot_id, zone_id):
                reserved.append(zone_id)
            else:
                for zid in reserved:
                    self.zone_locks[zid] = None  # roll back the partial reservation
                return False, zone_id
        return True, None

    def release_zones(self, robot_id, zone_ids, task_id=None):
        for zone_id in zone_ids:
            if self.zone_locks.get(zone_id) == robot_id:
                self.zone_locks[zone_id] = None
                self.log_event("ZONE", f"{zone_id} released by {robot_id}",
                               robot_id=robot_id, task_id=task_id, zone_id=zone_id)

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
            self.log_event("ZONE", f"{response.message} (manual)",
                           robot_id=request.robot_id, zone_id=request.zone_id)
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

    def set_wait_reason(self, task_id, reason, zone_id=None):
        task = self.task_registry[task_id]
        if task.get("wait_reason") != reason:  # log only when the reason changes
            task["wait_reason"] = reason
            self.log_event("TASK", f"{task_id} pending: {reason}", task_id=task_id, zone_id=zone_id)

    def try_assign(self, task_id):
        task = self.task_registry[task_id]
        if task["source"] not in LOCATIONS or task["destination"] not in LOCATIONS:
            self.get_logger().warn(
                f"{task_id}: unknown location '{task['source']}' or '{task['destination']}'"
            )
            return ASSIGN_INVALID

        src = LOCATIONS[task["source"]]
        dest = LOCATIONS[task["destination"]]
        candidates = self.available_robots(near=src)
        if not candidates:
            self.set_wait_reason(task_id, "waiting for an idle robot")
            return ASSIGN_RETRY

        blocked_zone = None
        # Nearest available robot whose whole route (current pose -> source ->
        # destination) can be reserved atomically; otherwise try the next nearest.
        for robot_id in candidates:
            robot = self.robot_registry[robot_id]
            pos = None if math.isnan(robot["x"]) else (robot["x"], robot["y"])
            zone_ids = self.zones_for_route([pos, src, dest])
            granted, blocked_zone = self.try_reserve_zones(robot_id, zone_ids)
            if granted:
                break
        else:
            self.set_wait_reason(
                task_id, f"waiting for {blocked_zone} (held by {self.zone_locks[blocked_zone]})",
                zone_id=blocked_zone,
            )
            return ASSIGN_RETRY

        for zone_id in zone_ids:
            self.log_event("ZONE", f"{zone_id} reserved by {robot_id} for {task_id}",
                           robot_id=robot_id, task_id=task_id, zone_id=zone_id)
        task["zones"] = zone_ids
        task["assigned_robot"] = robot_id
        task["status"] = "ASSIGNED"
        task["wait_reason"] = None
        task["assigned_at"] = time.time()
        robot["status"] = "BUSY"
        robot["available"] = False
        robot["current_task"] = task_id
        if not self.dispatch_task(task_id, robot_id, src, dest):
            # Robot's action server is not reachable: undo and keep the task queued.
            self.release_zones(robot_id, zone_ids, task_id)
            task.update(zones=[], assigned_robot=None, status="PENDING", assigned_at=None)
            robot["current_task"] = None
            self.set_wait_reason(task_id, f"{robot_id} action server unavailable")
            return ASSIGN_RETRY
        zones_text = ", ".join(zone_ids) if zone_ids else "no shared zones"
        self.log_event("TASK", f"{task_id} assigned to {robot_id} ({zones_text})",
                       robot_id=robot_id, task_id=task_id)
        return robot_id

    def dispatch_task(self, task_id, robot_id, src, dest):
        if robot_id not in self.task_clients:
            action_name = f"/{robot_id}/assign_task"
            self.task_clients[robot_id] = ActionClient(
                self, AssignTask, action_name
            )  # create local client for the selected robot (reused for later tasks)

        task_client = self.task_clients[robot_id]

        if not task_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(f"Action server for {robot_id} not available")
            return False

        goal = AssignTask.Goal()
        goal.task_id = task_id
        goal.source = self.task_registry[task_id]["source"]
        goal.destination = self.task_registry[task_id]["destination"]
        goal.source_x, goal.source_y = src
        goal.dest_x, goal.dest_y = dest

        future = task_client.send_goal_async(
            goal,
            feedback_callback=partial(
                self.callback_task_feedback, task_id=task_id, robot_id=robot_id
            ),
        )
        future.add_done_callback(
            partial(self.callback_assign_task, task_id=task_id, robot_id=robot_id)
        )
        return True

    def callback_assign_task(self, future, task_id, robot_id):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info(f"Task {task_id} accepted by {robot_id}")

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                partial(self.callback_task_result, task_id=task_id, robot_id=robot_id)
            )
        else:
            self.finish_task(task_id, robot_id, False, f"rejected by {robot_id}")

    def callback_task_feedback(self, feedback_msg, task_id, robot_id):
        feedback = feedback_msg.feedback
        task = self.task_registry[task_id]
        if task["status"] in TERMINAL_STATES:
            return
        if feedback.status != task["status"]:
            self.log_event("TASK", f"{task_id} | {robot_id} | {feedback.status}",
                           robot_id=robot_id, task_id=task_id)
        task["status"] = feedback.status
        task["progress"] = feedback.progress
        self.get_logger().debug(
            f"{task_id} | {robot_id} | " f"{feedback.status} | {feedback.progress}%"
        )

    def callback_task_result(self, future, task_id, robot_id):
        result = future.result().result
        self.finish_task(task_id, robot_id, result.success, result.message)

    def finish_task(self, task_id, robot_id, success, message):
        task = self.task_registry[task_id]
        self.release_zones(robot_id, task.get("zones", []), task_id)
        task["status"] = "COMPLETED" if success else "FAILED"
        if success:
            task["progress"] = 100.0
        task["message"] = message
        task["finished_at"] = time.time()
        robot = self.robot_registry[robot_id]
        robot["current_task"] = None
        robot["status"] = "IDLE"
        robot["available"] = True
        verb = "completed by" if success else "failed on"
        self.log_event("TASK", f"{task_id} {verb} {robot_id}: {message}",
                       robot_id=robot_id, task_id=task_id)

    def create_task(self, task_id, source, destination, priority):
        """Register a task and try to assign it. Returns (success, message)."""
        if task_id in self.task_registry:
            self.get_logger().warn(f"{task_id} already exists")
            return False, f"{task_id} already exists"
        unknown = [loc for loc in (source, destination) if loc not in LOCATIONS]
        if unknown:
            return False, f"{task_id}: unknown location {unknown[0]!r} (valid: {', '.join(LOCATIONS)})"
        if source == destination:
            return False, f"{task_id}: source and destination are the same"

        self.task_registry[task_id] = {
            "source": source,
            "destination": destination,
            "priority": priority,
            "status": "PENDING",
            "assigned_robot": None,
            "progress": 0.0,
            "zones": [],
            "created_at": time.time(),
        }
        self.log_event("TASK", f"{task_id} created: {source} -> {destination} (priority {priority})",
                       task_id=task_id)

        result = self.try_assign(task_id)
        if result == ASSIGN_RETRY:
            self.pending_tasks.append(task_id)
            return True, f"{task_id} created and queued ({self.task_registry[task_id]['wait_reason']})"
        return True, f"{task_id} assigned to {result}"

    def create_task_callback(
        self, request: CreateTask.Request, response: CreateTask.Response
    ):
        self.get_logger().info(f"Task creation request received: {request.task_id}")
        response.success, response.message = self.create_task(
            request.task_id, request.source, request.destination, request.priority
        )
        return response

    def process_pending_tasks(self):
        still_pending = []
        # Higher priority first; ties keep creation order.
        order = sorted(self.pending_tasks, key=lambda tid: -self.task_registry[tid]["priority"])
        for task_id in order:
            result = self.try_assign(task_id)
            if result == ASSIGN_RETRY:
                still_pending.append(task_id)
            elif result == ASSIGN_INVALID:
                self.task_registry[task_id]["status"] = "FAILED"
        self.pending_tasks = still_pending

    def sync_database(self):
        for request_id, task_id, source, destination, priority in self.db.pop_task_requests():
            success, message = self.create_task(task_id, source, destination, priority)
            self.db.mark_request(request_id, ("OK: " if success else "ERROR: ") + message)
        zones = [
            (zid, zx, zy, zr, self.zone_locks[zid],
             self.robot_registry.get(self.zone_locks[zid], {}).get("current_task"))
            for zid, zx, zy, zr in ZONES
        ]
        self.db.sync(self.robot_registry, self.task_registry, zones)

    def publish_status(self):
        self.get_logger().debug("Publishing message: Fleet manager is available")
        message = String()
        message.data = "Fleet manager is available"
        self.status_publisher_.publish(message)

    def publish_fleet_status(self):
        self.get_logger().debug("-----Fleet-Manager-----")
        for robot_id, robot_info in self.robot_registry.items():
            self.get_logger().debug(
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
                    self.log_event("ROBOT", f"{robot_id} has gone OFFLINE (no heartbeat for 2s)",
                                   robot_id=robot_id)


def main(args=None):
    rclpy.init(args=args)
    node = FleetManager()
    try:
        rclpy.spin(node)  # keep the node running until shutdown
    except KeyboardInterrupt:
        print(f"\nShutting down fleet manager")  # handle Ctrl+C gracefully
    finally:
        node.db.close()
        node.destroy_node()  # destroy the node and release its ROS2  resources.
        if rclpy.ok():  # check whether ROS2 is still running before shutdown
            rclpy.shutdown()


if __name__ == "__main__":
    main()
