"""SQLite persistence for the fleet manager.

The fleet manager is the only writer of fleet state; the Streamlit dashboard
reads these tables and inserts rows into task_requests, which the fleet
manager polls and turns into real tasks.
"""
import json
import os
import sqlite3
import time

SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    run_id INTEGER PRIMARY KEY AUTOINCREMENT,
    started_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS robots (
    robot_id TEXT PRIMARY KEY,
    robot_type TEXT,
    status TEXT,
    available INTEGER,
    current_task TEXT,
    x REAL,
    y REAL,
    last_seen REAL
);
CREATE TABLE IF NOT EXISTS zones (
    zone_id TEXT PRIMARY KEY,
    x REAL,
    y REAL,
    radius REAL,
    owner TEXT,
    task_id TEXT,
    updated_at REAL
);
CREATE TABLE IF NOT EXISTS tasks (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id INTEGER NOT NULL,
    task_id TEXT NOT NULL,
    source TEXT,
    destination TEXT,
    priority INTEGER,
    status TEXT,
    assigned_robot TEXT,
    progress REAL,
    zones TEXT,
    wait_reason TEXT,
    message TEXT,
    created_at REAL,
    assigned_at REAL,
    finished_at REAL,
    UNIQUE (run_id, task_id)
);
CREATE TABLE IF NOT EXISTS events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id INTEGER NOT NULL,
    ts REAL NOT NULL,
    category TEXT NOT NULL,
    robot_id TEXT,
    task_id TEXT,
    zone_id TEXT,
    message TEXT
);
CREATE TABLE IF NOT EXISTS locations (
    name TEXT PRIMARY KEY,
    x REAL,
    y REAL
);
CREATE TABLE IF NOT EXISTS task_requests (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT NOT NULL,
    source TEXT NOT NULL,
    destination TEXT NOT NULL,
    priority INTEGER DEFAULT 0,
    created_at REAL NOT NULL,
    processed_at REAL,
    response TEXT
);
"""

TERMINAL_STATES = ("COMPLETED", "FAILED")
DEFAULT_DB_PATH = os.path.expanduser("~/.awfms/awfms.db")


class FleetDB:
    def __init__(self, path=DEFAULT_DB_PATH):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.conn = sqlite3.connect(path, timeout=5.0)
        self.conn.execute("PRAGMA journal_mode=WAL")  # dashboard reads never block writes
        self.conn.executescript(SCHEMA)
        self.run_id = self._start_run()

    def _start_run(self):
        now = time.time()
        with self.conn:
            # Live state belongs to the current run only; history is kept.
            self.conn.execute("DELETE FROM robots")
            self.conn.execute("DELETE FROM zones")
            self.conn.execute(
                "UPDATE tasks SET status='FAILED', message='fleet manager restarted', "
                "finished_at=? WHERE status NOT IN (?, ?)", (now, *TERMINAL_STATES)
            )
            self.conn.execute(
                "UPDATE task_requests SET processed_at=?, response='discarded: fleet manager restarted' "
                "WHERE processed_at IS NULL", (now,)
            )
            return self.conn.execute(
                "INSERT INTO runs (started_at) VALUES (?)", (now,)
            ).lastrowid

    def set_locations(self, locations):
        with self.conn:
            self.conn.execute("DELETE FROM locations")
            self.conn.executemany(
                "INSERT INTO locations VALUES (?, ?, ?)",
                [(name, x, y) for name, (x, y) in locations.items()],
            )

    def log_event(self, category, message, robot_id=None, task_id=None, zone_id=None):
        with self.conn:
            self.conn.execute(
                "INSERT INTO events (run_id, ts, category, robot_id, task_id, zone_id, message) "
                "VALUES (?, ?, ?, ?, ?, ?, ?)",
                (self.run_id, time.time(), category, robot_id, task_id, zone_id, message),
            )

    def sync(self, robots, tasks, zones):
        """Write a snapshot of the in-memory registries in one transaction.

        robots: {robot_id: {type, status, available, current_task, x, y, last_seen}}
        tasks: {task_id: {source, destination, priority, status, assigned_robot,
                progress, zones, wait_reason, message, created_at, assigned_at, finished_at}}
        zones: [(zone_id, x, y, radius, owner, task_id)]
        """
        now = time.time()
        with self.conn:
            self.conn.executemany(
                "INSERT INTO robots VALUES (?, ?, ?, ?, ?, ?, ?, ?) ON CONFLICT(robot_id) DO UPDATE SET "
                "robot_type=excluded.robot_type, status=excluded.status, available=excluded.available, "
                "current_task=excluded.current_task, x=excluded.x, y=excluded.y, last_seen=excluded.last_seen",
                [
                    (rid, r["type"], r["status"], int(r["available"]), r.get("current_task"),
                     r.get("x"), r.get("y"), r.get("last_seen_wall"))
                    for rid, r in robots.items()
                ],
            )
            self.conn.executemany(
                "INSERT INTO tasks (run_id, task_id, source, destination, priority, status, assigned_robot, "
                "progress, zones, wait_reason, message, created_at, assigned_at, finished_at) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?) ON CONFLICT(run_id, task_id) DO UPDATE SET "
                "status=excluded.status, assigned_robot=excluded.assigned_robot, progress=excluded.progress, "
                "zones=excluded.zones, wait_reason=excluded.wait_reason, message=excluded.message, "
                "assigned_at=excluded.assigned_at, finished_at=excluded.finished_at",
                [
                    (self.run_id, tid, t["source"], t["destination"], t["priority"], t["status"],
                     t.get("assigned_robot"), t.get("progress", 0.0), json.dumps(t.get("zones", [])),
                     t.get("wait_reason"), t.get("message"), t.get("created_at"),
                     t.get("assigned_at"), t.get("finished_at"))
                    for tid, t in tasks.items()
                ],
            )
            self.conn.executemany(
                "INSERT INTO zones VALUES (?, ?, ?, ?, ?, ?, ?) ON CONFLICT(zone_id) DO UPDATE SET "
                "owner=excluded.owner, task_id=excluded.task_id, updated_at=excluded.updated_at",
                [(*z, now) for z in zones],
            )

    def pop_task_requests(self):
        """Return unprocessed dashboard task requests (oldest first)."""
        return self.conn.execute(
            "SELECT id, task_id, source, destination, priority FROM task_requests "
            "WHERE processed_at IS NULL ORDER BY id"
        ).fetchall()

    def mark_request(self, request_id, response):
        with self.conn:
            self.conn.execute(
                "UPDATE task_requests SET processed_at=?, response=? WHERE id=?",
                (time.time(), response, request_id),
            )

    def close(self):
        self.conn.close()
