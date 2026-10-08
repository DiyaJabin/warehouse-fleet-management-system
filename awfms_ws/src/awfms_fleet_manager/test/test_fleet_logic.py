"""Unit tests for zone planning, atomic reservation, task lifecycle and SQLite logging.

These run the real FleetManager node without Gazebo/Nav2: only the action
dispatch to robot nodes is stubbed out.
"""
import json
import sqlite3
import pytest
import rclpy
from awfms_interfaces.msg import RobotStatus
from awfms_interfaces.srv import RegisterRobot
from awfms_fleet_manager.fleet_db import FleetDB
from awfms_fleet_manager.fleet_manager import FleetManager, LOCATIONS, _dist_point_to_segment


@pytest.fixture
def fm(tmp_path):
    db_path = str(tmp_path / "test.db")
    rclpy.init(args=["--ros-args", "-p", f"db_path:={db_path}"])
    node = FleetManager()
    node.dispatched = []
    node.dispatch_task = lambda task_id, robot_id, src, dest: node.dispatched.append((task_id, robot_id)) or True
    yield node
    node.db.close()
    node.destroy_node()
    rclpy.shutdown()


def add_robot(fm, robot_id, x, y):
    req = RegisterRobot.Request(robot_id=robot_id, robot_type="AGV")
    fm.register_robot_callback(req, RegisterRobot.Response())
    fm.callback_robot_status(RobotStatus(robot_id=robot_id, status="IDLE", x=x, y=y))


def test_point_to_segment_distance():
    assert _dist_point_to_segment(0, 1, -1, 0, 1, 0) == pytest.approx(1.0)
    assert _dist_point_to_segment(3, 0, -1, 0, 1, 0) == pytest.approx(2.0)  # beyond segment end
    assert _dist_point_to_segment(1, 1, 0, 0, 0, 0) == pytest.approx(2 ** 0.5)  # degenerate segment


def test_zones_for_route(fm):
    L = LOCATIONS
    assert fm.zones_for_route([L["pickup"], L["dropoff"]]) == ["corridor_west", "corridor_center", "corridor_east"]
    assert fm.zones_for_route([L["bay_nw"], L["bay_ne"]]) == []
    assert fm.zones_for_route([(0.0, 0.0), L["pickup"], L["bay_nw"]]) == ["corridor_west", "corridor_center"]
    assert len(fm.zones_for_route([None, L["bay_nw"], L["bay_ne"]])) == 3  # unknown pose -> conservative


def test_atomic_reservation_rolls_back(fm):
    assert fm.try_reserve_zones("robot_1", ["corridor_center"]) == (True, None)
    assert fm.try_reserve_zones("robot_2", ["corridor_west", "corridor_center"]) == (False, "corridor_center")
    assert fm.zone_locks["corridor_west"] is None  # partial reservation was rolled back
    assert fm.zone_locks["corridor_center"] == "robot_1"


def test_contention_queue_and_release(fm):
    add_robot(fm, "robot_1", -8.0, 0.0)
    add_robot(fm, "robot_2", 8.0, 0.0)
    ok, msg = fm.create_task("t1", "pickup", "dropoff", 1)
    assert ok and "assigned to robot_1" in msg
    ok, msg = fm.create_task("t2", "dropoff", "pickup", 1)
    assert ok and "queued" in msg and "held by robot_1" in msg
    assert fm.task_registry["t2"]["status"] == "PENDING"

    fm.process_pending_tasks()  # still blocked
    assert fm.pending_tasks == ["t2"]

    fm.finish_task("t1", "robot_1", True, "done")
    assert all(owner is None for owner in fm.zone_locks.values())
    fm.process_pending_tasks()
    assert fm.pending_tasks == []
    assert fm.task_registry["t2"]["status"] == "ASSIGNED"
    assert fm.dispatched == [("t1", "robot_1"), ("t2", "robot_2")]  # robot_2 is nearest to dropoff


def test_nearest_robot_with_clear_route_is_chosen(fm):
    # Spawn layout: every robot is parked in the central corridor.
    for rid, x in [("robot_1", 0.0), ("robot_2", 2.0), ("robot_3", -2.0), ("robot_4", 4.0)]:
        add_robot(fm, rid, x, 0.0)
    fm.create_task("t1", "pickup", "bay_nw", 1)
    fm.create_task("t2", "dropoff", "bay_se", 1)
    assert fm.dispatched == [("t1", "robot_3"), ("t2", "robot_4")]
    assert fm.task_registry["t1"]["zones"] == ["corridor_west", "corridor_center"]
    assert fm.task_registry["t2"]["zones"] == ["corridor_east"]  # disjoint zones -> both run at once


def test_failure_releases_zones(fm):
    add_robot(fm, "robot_1", -8.0, 0.0)
    fm.create_task("t1", "pickup", "dropoff", 1)
    fm.finish_task("t1", "robot_1", False, "navigation failed")
    assert fm.task_registry["t1"]["status"] == "FAILED"
    assert all(owner is None for owner in fm.zone_locks.values())
    assert fm.robot_registry["robot_1"]["available"]


def test_non_conflicting_tasks_run_concurrently(fm):
    add_robot(fm, "robot_1", -7.5, 5.0)
    add_robot(fm, "robot_2", -7.5, -5.0)
    fm.create_task("t1", "bay_nw", "bay_ne", 1)
    fm.create_task("t2", "bay_sw", "bay_se", 1)
    assert [r for _, r in fm.dispatched] == ["robot_1", "robot_2"]
    assert fm.task_registry["t1"]["zones"] == [] and fm.task_registry["t2"]["zones"] == []


def test_stale_idle_heartbeat_does_not_free_busy_robot(fm):
    add_robot(fm, "robot_1", -8.0, 0.0)
    fm.create_task("t1", "pickup", "bay_nw", 1)
    fm.callback_robot_status(RobotStatus(robot_id="robot_1", status="IDLE", x=-8.0, y=0.0))
    assert not fm.robot_registry["robot_1"]["available"]
    ok, msg = fm.create_task("t2", "bay_sw", "bay_se", 1)
    assert "waiting for an idle robot" in msg


def test_invalid_tasks_rejected(fm):
    assert fm.create_task("t1", "pickup", "nowhere", 1)[0] is False
    assert fm.create_task("t2", "pickup", "pickup", 1)[0] is False
    fm.create_task("t3", "pickup", "dropoff", 1)
    assert fm.create_task("t3", "pickup", "dropoff", 1)[0] is False  # duplicate id


def test_database_sync_and_dashboard_requests(fm, tmp_path):
    add_robot(fm, "robot_1", -8.0, 0.0)
    fm.db.conn.execute(
        "INSERT INTO task_requests (task_id, source, destination, priority, created_at) "
        "VALUES ('dash_1', 'pickup', 'dropoff', 2, 0)"
    )
    fm.db.conn.commit()
    fm.sync_database()

    conn = sqlite3.connect(str(tmp_path / "test.db"))
    status, robot, zones = conn.execute("SELECT status, assigned_robot, zones FROM tasks WHERE task_id='dash_1'").fetchone()
    assert (status, robot) == ("ASSIGNED", "robot_1")
    assert json.loads(zones) == ["corridor_west", "corridor_center", "corridor_east"]
    assert conn.execute("SELECT owner, task_id FROM zones WHERE zone_id='corridor_center'").fetchone() == ("robot_1", "dash_1")
    assert conn.execute("SELECT response FROM task_requests").fetchone()[0].startswith("OK")
    assert conn.execute("SELECT current_task FROM robots").fetchone()[0] == "dash_1"
    categories = {c for (c,) in conn.execute("SELECT category FROM events")}
    assert {"SYSTEM", "ROBOT", "TASK", "ZONE"} <= categories
    conn.close()


def test_restart_fails_unfinished_tasks(tmp_path):
    path = str(tmp_path / "restart.db")
    db = FleetDB(path)
    db.sync({}, {"t1": {"source": "pickup", "destination": "dropoff", "priority": 1, "status": "TO_SOURCE"}}, [])
    db.close()
    db = FleetDB(path)
    assert db.run_id == 2
    assert db.conn.execute("SELECT status FROM tasks").fetchone()[0] == "FAILED"
    db.close()
