"""AWFMS Streamlit dashboard.

Reads fleet state written by the fleet manager to SQLite and submits new tasks
through the task_requests table (the fleet manager polls it once a second).
Run: streamlit run dashboard/app.py
"""
import json
import os
import sqlite3
import time
import altair as alt
import pandas as pd
import streamlit as st

DB_PATH = os.environ.get("AWFMS_DB", os.path.expanduser("~/.awfms/awfms.db"))
ROBOT_COLORS = {"robot_1": "#2a78d6", "robot_2": "#eb6834", "robot_3": "#1baf7a", "robot_4": "#e87ba4"}
STATUS_ICON = {
    "IDLE": "🟢 IDLE", "MOVING": "🔵 MOVING", "BUSY": "🔵 BUSY", "OFFLINE": "🔴 OFFLINE", "UNKNOWN": "⚪ UNKNOWN",
    "PENDING": "🟡 PENDING", "ASSIGNED": "🔵 ASSIGNED", "TO_SOURCE": "🔵 TO_SOURCE",
    "TO_DESTINATION": "🔵 TO_DESTINATION", "COMPLETED": "✅ COMPLETED", "FAILED": "❌ FAILED",
}
ACTIVE_STATES = ("ASSIGNED", "TO_SOURCE", "TO_DESTINATION")
# Static warehouse geometry from warehouse.sdf (for drawing only).
SHELVES = [(x, y) for y in (3.0, -3.0) for x in (-5.0, 0.0, 5.0)]
# Same batches as `ros2 run awfms_fleet_manager demo_tasks <name>`; run in this order.
SCENARIOS = {
    "1 · Normal: two robots leave the corridor": [("pickup", "bay_nw", 1), ("dropoff", "bay_se", 1)],
    "2 · Contention: opposite corridor runs": [("pickup", "dropoff", 1), ("dropoff", "pickup", 1)],
    "3 · Concurrent: disjoint routes": [("bay_se", "bay_sw", 1), ("pickup", "bay_nw", 1), ("dropoff", "bay_ne", 1)],
    "4 · Failure: unreachable destination": [("pickup", "fault_test", 1)],
}

st.set_page_config(page_title="AWFMS Fleet Dashboard", page_icon="🤖", layout="wide")


def connect(readonly=True):
    if readonly:
        return sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True, timeout=5.0)
    return sqlite3.connect(DB_PATH, timeout=5.0)


def query(sql, params=()):
    with connect() as conn:
        return pd.read_sql_query(sql, conn, params=params)


def submit_tasks(tasks):
    now = time.time()
    with connect(readonly=False) as conn:
        for i, (source, destination, priority) in enumerate(tasks, start=1):
            task_id = f"dash_{time.strftime('%H%M%S')}_{i}"
            conn.execute(
                "INSERT INTO task_requests (task_id, source, destination, priority, created_at) VALUES (?, ?, ?, ?, ?)",
                (task_id, source, destination, priority, now),
            )


def ago(ts):
    if ts is None or pd.isna(ts):
        return "—"
    s = max(0, time.time() - ts)
    return f"{s:.0f}s ago" if s < 120 else f"{s / 60:.0f}m ago"


def clock(ts):
    return "—" if ts is None or pd.isna(ts) else time.strftime("%H:%M:%S", time.localtime(ts))


if not os.path.exists(DB_PATH):
    st.error(f"Database not found at `{DB_PATH}`. Start the fleet manager first (it creates the database).")
    st.stop()

runs = query("SELECT run_id, started_at FROM runs ORDER BY run_id DESC")
locations = query("SELECT name, x, y FROM locations ORDER BY name")

# ---------------------------------------------------------------- sidebar
with st.sidebar:
    st.header("🤖 AWFMS")
    st.caption(f"Database: `{DB_PATH}`")
    run_labels = {r.run_id: f"Run {r.run_id} — started {time.strftime('%d %b %H:%M', time.localtime(r.started_at))}"
                  for r in runs.itertuples()}
    run_id = st.selectbox("Fleet manager run", list(run_labels), format_func=run_labels.get)
    latest_run = run_id == runs.run_id.iloc[0]
    if not latest_run:
        st.info("Viewing history: live robot and zone state belong to the latest run.")
    refresh = st.select_slider("Auto-refresh", options=[0, 1, 2, 5], value=1, format_func=lambda v: "off" if v == 0 else f"{v}s")

    st.subheader("Create task")
    names = [n for n in locations.name if n != "fault_test"] + ["fault_test"]
    with st.form("create_task", clear_on_submit=False, border=False):
        source = st.selectbox("Source", names, index=names.index("pickup") if "pickup" in names else 0)
        destination = st.selectbox("Destination", names, index=names.index("dropoff") if "dropoff" in names else 0)
        priority = st.number_input("Priority (higher first)", min_value=0, max_value=10, value=1)
        if st.form_submit_button("Submit task", type="primary", width="stretch", disabled=not latest_run):
            if source == destination:
                st.warning("Source and destination must differ.")
            else:
                submit_tasks([(source, destination, int(priority))])
                st.success(f"Requested {source} → {destination}")

    st.subheader("Demo scenarios")
    for label, tasks in SCENARIOS.items():
        if st.button(label, width="stretch", disabled=not latest_run):
            submit_tasks(tasks)
            st.success(f"Submitted {len(tasks)} task(s)")
    st.caption("`fault_test` is intentionally inside a shelf to demonstrate failure cleanup.")


# ---------------------------------------------------------------- map
def warehouse_map(robots, zones):
    dark = getattr(st.context, "theme", None) is not None and st.context.theme.type == "dark"
    ink, muted = ("#ffffff", "#c3c2b7") if dark else ("#16324f", "#4a6585")
    shelves = pd.DataFrame([{"x1": x - 1.05, "x2": x + 1.05, "y1": y - 0.5, "y2": y + 0.5} for x, y in SHELVES])
    base = alt.Chart().properties(height=420)
    x_scale = alt.Scale(domain=[-10, 10], nice=False)
    y_scale = alt.Scale(domain=[-7, 7], nice=False)
    walls = alt.Chart(pd.DataFrame([{"x1": -9.9, "x2": 9.9, "y1": -6.8, "y2": 6.8}])).mark_rect(
        fill="transparent", stroke="#8fa9c8", strokeWidth=2
    ).encode(x=alt.X("x1:Q", scale=x_scale, title=None), x2="x2", y=alt.Y("y1:Q", scale=y_scale, title=None), y2="y2")
    shelf_layer = alt.Chart(shelves).mark_rect(fill="#b4c6dc", cornerRadius=2).encode(
        x=alt.X("x1:Q", scale=x_scale), x2="x2", y=alt.Y("y1:Q", scale=y_scale), y2="y2"
    )
    z = zones.copy()
    z["state"] = z.owner.apply(lambda o: f"reserved by {o}" if o else "free")
    z["fill"] = z.owner.apply(lambda o: ROBOT_COLORS.get(o, "#d03b3b") if o else "#c4d8f0")
    # Altair sizes points by area in px^2: convert zone radius (m) to pixels.
    px_per_m = 420 / 14
    z["size"] = (2 * z.radius * px_per_m) ** 2
    zone_layer = alt.Chart(z).mark_point(shape="circle", filled=True, opacity=0.35, stroke=muted, strokeDash=[4, 3]).encode(
        x=alt.X("x:Q", scale=x_scale), y=alt.Y("y:Q", scale=y_scale), size=alt.Size("size:Q", scale=None, legend=None),
        color=alt.Color("fill:N", scale=None), tooltip=[alt.Tooltip("zone_id", title="Zone"), alt.Tooltip("state", title="State"),
                                                        alt.Tooltip("task_id", title="Task")],
    )
    zone_labels = alt.Chart(z).mark_text(dy=-48, fontSize=11, color=muted).encode(
        x=alt.X("x:Q", scale=x_scale), y=alt.Y("y:Q", scale=y_scale), text="zone_id")
    loc = locations[locations.name != "fault_test"]
    loc_layer = alt.Chart(loc).mark_point(shape="square", size=160, filled=False, color=muted, strokeWidth=2).encode(
        x=alt.X("x:Q", scale=x_scale), y=alt.Y("y:Q", scale=y_scale), tooltip=["name"])
    loc_labels = alt.Chart(loc).mark_text(dy=16, fontSize=11, color=muted).encode(
        x=alt.X("x:Q", scale=x_scale), y=alt.Y("y:Q", scale=y_scale), text="name")
    layers = [walls, shelf_layer, zone_layer, zone_labels, loc_layer, loc_labels]
    r = robots.dropna(subset=["x", "y"]).copy()
    if not r.empty:
        r["fill"] = r.robot_id.map(ROBOT_COLORS).fillna("#4a3aa7")
        r["task"] = r.current_task.fillna("—")
        robot_layer = alt.Chart(r).mark_point(shape="circle", filled=True, size=260, opacity=1, stroke="#ffffff", strokeWidth=2).encode(
            x=alt.X("x:Q", scale=x_scale), y=alt.Y("y:Q", scale=y_scale), color=alt.Color("fill:N", scale=None),
            tooltip=[alt.Tooltip("robot_id", title="Robot"), "status", "task", alt.Tooltip("x", format=".2f"), alt.Tooltip("y", format=".2f")],
        )
        robot_labels = alt.Chart(r).mark_text(dx=14, align="left", fontSize=12, fontWeight="bold", color=ink).encode(
            x=alt.X("x:Q", scale=x_scale), y=alt.Y("y:Q", scale=y_scale), text="robot_id")
        layers += [robot_layer, robot_labels]
    return alt.layer(*layers).properties(height=420).configure_axis(grid=False, labels=False, ticks=False, domain=False).configure_view(stroke=None)


# ---------------------------------------------------------------- live page
@st.fragment(run_every=refresh or None)
def live():
    robots = query("SELECT * FROM robots ORDER BY robot_id") if latest_run else pd.DataFrame(
        columns=["robot_id", "robot_type", "status", "available", "current_task", "x", "y", "last_seen"])
    zones = query("SELECT * FROM zones ORDER BY x")
    tasks = query("SELECT * FROM tasks WHERE run_id=? ORDER BY id DESC", (run_id,))
    events = query("SELECT * FROM events WHERE run_id=? ORDER BY id DESC LIMIT 300", (run_id,))
    requests = query("SELECT * FROM task_requests ORDER BY id DESC LIMIT 5")

    st.title("Autonomous Warehouse Fleet Management")
    st.caption(f"Centralized task allocation and atomic zone reservation for a four-robot Nav2 fleet · updated {time.strftime('%H:%M:%S')}")

    online = (robots.status != "OFFLINE").sum()
    busy = robots.status.isin(["MOVING", "BUSY"]).sum()
    counts = tasks.status.value_counts()
    # Card widths follow label length so labels like "Robots online" are not truncated.
    cols = st.columns([1.45, 0.85, 0.85, 1.35, 1.1, 1.3, 0.9])
    cols[0].metric("Robots online", f"{online}/{len(robots)}", border=True)
    cols[1].metric("Idle", int((robots.status == "IDLE").sum()), border=True)
    cols[2].metric("Busy", int(busy), border=True)
    cols[3].metric("Active tasks", int(counts.reindex(ACTIVE_STATES).fillna(0).sum()), border=True)
    cols[4].metric("Pending", int(counts.get("PENDING", 0)), border=True)
    cols[5].metric("Completed", int(counts.get("COMPLETED", 0)), border=True)
    cols[6].metric("Failed", int(counts.get("FAILED", 0)), border=True)

    left, right = st.columns([3, 2])
    with left:
        st.subheader("Warehouse map")
        st.altair_chart(warehouse_map(robots, zones), width="stretch")
        st.caption("Circles are shared corridor zones (pale blue = free, robot colour = reserved). Squares are named locations.")
    with right:
        st.subheader("Zone reservations")
        for z in zones.itertuples():
            if z.owner:
                st.error(f"🔒 **{z.zone_id}** — reserved by **{z.owner}**" + (f" for `{z.task_id}`" if z.task_id else ""))
            else:
                st.success(f"🔓 **{z.zone_id}** — free")
        st.subheader("Robots")
        if robots.empty:
            st.info("No robots registered yet.")
        else:
            st.dataframe(pd.DataFrame({
                "Robot": robots.robot_id,
                "State": robots.status.map(lambda s: STATUS_ICON.get(s, s)),
                "Available": robots.available.map({1: "yes", 0: "no"}),
                "Current task": robots.current_task.fillna("—"),
                "Pose (x, y)": [f"({x:.1f}, {y:.1f})" if pd.notna(x) else "not localized" for x, y in zip(robots.x, robots.y)],
                "Heartbeat": robots.last_seen.map(ago),
            }), hide_index=True, width="stretch")

    st.subheader("Tasks")
    if tasks.empty:
        st.info("No tasks in this run yet — create one from the sidebar or run `ros2 run awfms_fleet_manager demo_tasks normal`.")
    else:
        wait = (tasks.assigned_at.fillna(time.time()) - tasks.created_at).where(tasks.status != "FAILED", None)
        # Display-only shortening: the column headers already say "Waiting for" and all zones are corridor zones.
        st.dataframe(pd.DataFrame({
            "Task": tasks.task_id,
            "Route": tasks.source + " → " + tasks.destination,
            "Priority": tasks.priority,
            "Status": tasks.status.map(lambda s: STATUS_ICON.get(s, s)),
            "Robot": tasks.assigned_robot.fillna("—"),
            "Waiting for": tasks.wait_reason.fillna("").str.replace(r"^waiting for ", "", regex=True).replace("", "—"),
            "Progress": tasks.progress.fillna(0.0),
            "Zones": tasks.zones.map(lambda z: " · ".join(n.removeprefix("corridor_") for n in json.loads(z)) if z else "")
                                .replace("", "—"),
            "Created": tasks.created_at.map(clock),
            "Assigned": tasks.assigned_at.map(clock),
            "Finished": tasks.finished_at.map(clock),
            "Wait (s)": wait.round(1),
            "Message": tasks.message.fillna(""),
        }), hide_index=True, width="stretch", row_height=32, column_config={
            "Task": st.column_config.TextColumn(width="medium", pinned=True),
            "Route": st.column_config.TextColumn(width="medium"),
            "Priority": st.column_config.NumberColumn("Prio", width="small", help="Task priority (higher first)"),
            "Status": st.column_config.TextColumn(width="medium"),
            "Robot": st.column_config.TextColumn(width="small"),
            "Waiting for": st.column_config.TextColumn(width="medium", help="Why a PENDING task has not been dispatched"),
            "Progress": st.column_config.ProgressColumn("Progress", min_value=0, max_value=100, format="%.0f%%", width="small"),
            "Zones": st.column_config.TextColumn(width="medium", help="Corridor zones reserved for the task (west · center · east)"),
            "Created": st.column_config.TextColumn(width="small"),
            "Assigned": st.column_config.TextColumn(width="small"),
            "Finished": st.column_config.TextColumn(width="small"),
            "Wait (s)": st.column_config.NumberColumn(width="small", help="Queue wait from creation to assignment"),
            "Message": st.column_config.TextColumn(width="large"),
        })

    c1, c2 = st.columns(2)
    with c1:
        st.subheader("Task outcomes per robot")
        done = tasks[tasks.assigned_robot.notna()]
        if done.empty:
            st.caption("No assigned tasks yet.")
        else:
            outcome = done.assign(outcome=done.status.where(done.status.isin(["COMPLETED", "FAILED"]), "ACTIVE"))
            chart = alt.Chart(outcome).mark_bar(cornerRadiusEnd=4).encode(
                y=alt.Y("assigned_robot:N", title=None), x=alt.X("count():Q", title="Tasks", axis=alt.Axis(tickMinStep=1)),
                color=alt.Color("outcome:N", title="Outcome", scale=alt.Scale(
                    domain=["COMPLETED", "ACTIVE", "FAILED"], range=["#0ca30c", "#2a78d6", "#d03b3b"])),
                tooltip=["assigned_robot", "outcome", "count()"],
            ).properties(height=200)
            st.altair_chart(chart, width="stretch")
    with c2:
        st.subheader("Zone activity")
        zev = events[events.zone_id.notna()].copy()
        if zev.empty:
            st.caption("No zone events yet.")
        else:
            zev["kind"] = zev.message.map(lambda m: "reserved" if " reserved " in m or "granted" in m
                                          else "released" if "released" in m else "blocked a task")
            chart = alt.Chart(zev).mark_bar(cornerRadiusEnd=4).encode(
                y=alt.Y("zone_id:N", title=None), x=alt.X("count():Q", title="Events", axis=alt.Axis(tickMinStep=1)),
                yOffset="kind:N",
                color=alt.Color("kind:N", title="Event", scale=alt.Scale(
                    domain=["reserved", "released", "blocked a task"], range=["#2a78d6", "#1baf7a", "#eb6834"])),
                tooltip=["zone_id", "kind", "count()"],
            ).properties(height=200)
            st.altair_chart(chart, width="stretch")

    st.subheader("Event log")
    cats = st.multiselect("Categories", ["TASK", "ZONE", "ROBOT", "SYSTEM"], default=["TASK", "ZONE", "ROBOT", "SYSTEM"],
                          label_visibility="collapsed")
    ev = events[events.category.isin(cats)].head(100)
    st.dataframe(pd.DataFrame({"Time": ev.ts.map(clock), "Category": ev.category, "Robot": ev.robot_id.fillna(""),
                               "Task": ev.task_id.fillna(""), "Zone": ev.zone_id.fillna(""), "Event": ev.message}),
                 hide_index=True, width="stretch", height=320)

    if not requests.empty:
        with st.expander("Recent dashboard task requests"):
            st.dataframe(requests.assign(created_at=requests.created_at.map(clock), processed_at=requests.processed_at.map(clock))
                         [["task_id", "source", "destination", "priority", "created_at", "processed_at", "response"]],
                         hide_index=True, width="stretch")


live()
