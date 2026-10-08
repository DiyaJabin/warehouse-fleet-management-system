#!/usr/bin/env python3
"""Submit demonstration task batches through /fleet_manager/create_task.

Usage: ros2 run awfms_fleet_manager demo_tasks <scenario>
"""
import sys
import time
import rclpy
from rclpy.node import Node
from awfms_interfaces.srv import CreateTask

# (source, destination, priority)
SCENARIOS = {
    # Designed to run in this order from the spawn layout (all robots parked in
    # the central corridor); assignment picks the nearest idle robot.
    # 1. Two robots leave the corridor at once using disjoint zones (west+center / east).
    "normal": [("pickup", "bay_nw", 1), ("dropoff", "bay_se", 1)],
    # 2. Opposite full-corridor runs: both need every corridor zone, so the
    #    second task stays PENDING until the first releases its zones.
    "contention": [("pickup", "dropoff", 1), ("dropoff", "pickup", 1)],
    # 3. South aisle, west end and corridor runs in parallel with no shared zones.
    "concurrent": [("bay_se", "bay_sw", 1), ("pickup", "bay_nw", 1), ("dropoff", "bay_ne", 1)],
    # 4. Unreachable destination: task FAILS, its zones are released.
    "failure": [("pickup", "fault_test", 1)],
}


class DemoClient(Node):
    def __init__(self):
        super().__init__("awfms_demo_tasks")
        self.client = self.create_client(CreateTask, "/fleet_manager/create_task")

    def submit(self, task_id, source, destination, priority):
        request = CreateTask.Request(task_id=task_id, source=source, destination=destination, priority=priority)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None:
            return False, "no response from fleet manager"
        return response.success, response.message


def main(args=None):
    scenario = sys.argv[1] if len(sys.argv) > 1 else ""
    if scenario not in SCENARIOS:
        print(f"usage: demo_tasks <{'|'.join(SCENARIOS)}>")
        return
    rclpy.init(args=args)
    node = DemoClient()
    if not node.client.wait_for_service(timeout_sec=10.0):
        print("fleet manager create_task service not available")
    else:
        stamp = time.strftime("%H%M%S")
        for i, (source, destination, priority) in enumerate(SCENARIOS[scenario], start=1):
            ok, message = node.submit(f"{scenario}_{stamp}_{i}", source, destination, priority)
            print(("OK   " if ok else "FAIL ") + message)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
