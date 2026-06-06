import math
import threading
import time

import yaml

import rclpy
from rclpy.executors import MultiThreadedExecutor

import yasmin
from yasmin import Blackboard, StateMachine, CbState
from yasmin_ros import ActionState, set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from geometry_msgs.msg import (
    Pose, PoseArray, PointStamped, Twist, PoseWithCovarianceStamped)
from std_msgs.msg import Bool, Int32
from nav2_msgs.action import NavigateToPose, ComputePathToPose

HAS_NEXT = "has_next"
NO_NEXT = "no_next"
KEEP_GOING = "keep_going"
DONE = "done"
FOUND = "found"
NONE = "none"
COLLECTED = "collected"
NOT_COLLECTED = "not_collected"
RETRY = "retry"
GIVE_UP = "give_up"
MAX_REACHED = "max_reached"
MORE_DUPLO = "more_duplo"
DUPLO_FOUND = "duplo_found"
RESET = "reset"
REACHABLE = "reachable"
UNREACHABLE = "unreachable"


def node_key(wp):
    return (round(float(wp[0]), 3), round(float(wp[1]), 3))


def make_pose(x: float, y: float, yaw: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


# Callback function for /duplo_map PoseArray of duplos
def on_duplo_map(msg: PoseArray, blackboard: Blackboard) -> None:
    blackboard["duplo_map"] = msg

# Callback function for /amcl_pose robot position
def on_amcl_pose(msg: PoseWithCovarianceStamped, blackboard: Blackboard) -> None:
    blackboard["robot_xy"] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

# Callback function for the Arduino "duplos caught" counter (cumulative since boot)
def on_caught_total(msg: Int32, blackboard: Blackboard) -> None:
    blackboard["caught_total"] = int(msg.data)


class Nav2State(ActionState):
    def __init__(self, timeout_s: float = 10.0) -> None:
        super().__init__(
            NavigateToPose,
            "/navigate_to_pose",
            self.create_goal_handler,
            [SUCCEED, ABORT, CANCEL, DUPLO_FOUND],
            None,
        )
        self.timeout_s = float(timeout_s)

    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.pose = blackboard["pose"]
        goal.pose.header.frame_id = "map"
        return goal

    def execute(self, blackboard: Blackboard) -> str:
        timeout = blackboard.get("nav_timeout_s", self.timeout_s)
        deadline = time.time() + timeout
        tol = blackboard.get("duplo_match_tol", 0.20)

        initial_map = blackboard.get("duplo_map") or PoseArray()
        initial_positions = [
            (p.position.x, p.position.y) for p in initial_map.poses]

        stop_event = threading.Event()
        state = {"timed_out": False, "duplo_found": False}

        def watchdog():
            while not stop_event.is_set():
                if time.time() >= deadline:
                    state["timed_out"] = True
                    blackboard["node"].get_logger().warn(
                        f"Nav2 timeout after {timeout:.0f}s — cancelling")
                    self.cancel_state()
                    return
                if blackboard.get("interruptible", True):
                    current = blackboard.get("duplo_map") or PoseArray()
                    for p in current.poses:
                        x, y = p.position.x, p.position.y
                        new = not any(
                            math.hypot(x - sx, y - sy) < tol
                            for sx, sy in initial_positions)
                        if new:
                            state["duplo_found"] = True
                            blackboard["node"].get_logger().info(
                                f"NEW duplo @ ({x:.2f}, {y:.2f}) — interrupting nav")
                            self.cancel_state()
                            return
                time.sleep(0.2)

        watchdog_thread = threading.Thread(target=watchdog, daemon=True)
        watchdog_thread.start()
        try:
            outcome = super().execute(blackboard)
        finally:
            stop_event.set()

        if state["duplo_found"]:
            return DUPLO_FOUND
        if state["timed_out"]:
            return ABORT
        return outcome


class IsReachableState(ActionState):
    def __init__(self, timeout_s: float = 10.0) -> None:
        super().__init__(
            ComputePathToPose,
            "/compute_path_to_pose",
            self.create_goal_handler,
            [REACHABLE, UNREACHABLE, ABORT, CANCEL],
            self.handle_result,
        )
        self.timeout_s = float(timeout_s)

    def create_goal_handler(self, blackboard: Blackboard) -> ComputePathToPose.Goal:
        goal = ComputePathToPose.Goal()
        goal.goal.pose = blackboard["pose"]
        goal.goal.header.frame_id = "map"
        goal.use_start = False
        return goal

    def handle_result(self, blackboard: Blackboard, result) -> str:
        if result is None or not result.path.poses:
            blackboard["node"].get_logger().warn("IS_REACHABLE: no path found")
            return UNREACHABLE
        blackboard["node"].get_logger().info(
            f"IS_REACHABLE: path of {len(result.path.poses)} poses ok")
        return REACHABLE

    def execute(self, blackboard: Blackboard) -> str:
        timeout = blackboard.get("plan_timeout_s", self.timeout_s)
        deadline = time.time() + timeout
        stop_event = threading.Event()
        timed_out = {"hit": False}

        def watchdog():
            while not stop_event.is_set():
                if time.time() >= deadline:
                    timed_out["hit"] = True
                    self.cancel_state()
                    return
                time.sleep(0.1)

        watchdog_thread = threading.Thread(target=watchdog, daemon=True)
        watchdog_thread.start()
        try:
            outcome = super().execute(blackboard)
        finally:
            stop_event.set()

        if timed_out["hit"]:
            blackboard["node"].get_logger().warn(
                f"IS_REACHABLE: plan timeout after {timeout:.0f}s — unreachable")
            return UNREACHABLE
        if outcome == ABORT:
            return UNREACHABLE
        return outcome


def init_grid(blackboard: Blackboard) -> str:
    """
    Initializes the grid by loading the waypoints file and other variables. 
    The blackboard is for the shared values among states. 
    """
    with open(blackboard["waypoints_file"]) as f:
        data = yaml.safe_load(f)
    blackboard["grid"] = list(data["waypoints"])
    blackboard["done_nodes"] = set()
    blackboard["failed_count"] = {}
    blackboard["current_node"] = None
    blackboard["carrying"] = 0
    blackboard["total_collected"] = 0
    blackboard["t_end"] = time.time() + blackboard["mission_time"]
    blackboard["interruptible"] = True
    blackboard["enable_pub"].publish(Bool(data=False))
    return SUCCEED


def _candidate_nodes(blackboard):
    """
    Defines the candidate nodes. That is, unvisited dones that 
    were not marked as failed. 
    """
    done = blackboard["done_nodes"]
    failed = blackboard["failed_count"]
    cap = blackboard["max_node_retries"]
    out = []
    for wp in blackboard["grid"]:
        key = node_key(wp)
        if key in done:
            continue
        if failed.get(key, 0) >= cap:
            continue
        out.append(wp)
    return out


def check_mission_alive(blackboard: Blackboard) -> str:
    """
    Check if the mission is still alive. 
    The mission is still alive if:
    - The time for exploration is not yet over 
    - All duplos in the zone we not yet picked up
    """
    if time.time() >= blackboard["t_end"]:
        blackboard["node"].get_logger().info("MISSION: time budget exhausted")
        return DONE
    if blackboard["total_collected"] >= blackboard["expected_duplos_in_zone"]:
        blackboard["node"].get_logger().info(
            f"MISSION: collected {blackboard['total_collected']}/"
            f"{blackboard['expected_duplos_in_zone']} — done")
        return DONE
    return KEEP_GOING


def select_target(blackboard: Blackboard) -> str:
    """
    Selects the next duplo to be catched. This is a pure greedy closest pick. 
    """
    duplo_map = blackboard.get("duplo_map")
    if duplo_map is None or not duplo_map.poses:
        return NONE
    rx, ry = blackboard.get("robot_xy", (0.0, 0.0))
    nearest = min(
        duplo_map.poses,
        key=lambda p: math.hypot(p.position.x - rx, p.position.y - ry))
    blackboard["target_xy"] = (nearest.position.x, nearest.position.y)
    blackboard["approach_retries"] = 0
    blackboard["node"].get_logger().info(
        f"SELECT_TARGET -> ({nearest.position.x:.2f}, {nearest.position.y:.2f})")
    return FOUND

def best_node(blackboard: Blackboard) -> str:
    """
    Best node selection algorithm that uses motion coherence. 
    It adds the following behavior:
    - prefers nodes that continue the current direction
    - produces smoother sweeps across the map
    """
    candidates = _candidate_nodes(blackboard)
    if not candidates:
        return NO_NEXT

    rx, ry = blackboard.get("robot_xy", (0.0, 0.0))
    last = blackboard.get("last_node_xy")

    def score(w):
        x, y, _ = w
        dx, dy = x - rx, y - ry
        dist_score = math.hypot(dx, dy)

        # If we have a previous node, enforce directional continuity
        if last is not None:
            lx, ly = last
            v1 = (lx - rx, ly - ry)
            v2 = (dx, dy)

            norm1 = math.hypot(*v1) + 1e-6
            norm2 = math.hypot(*v2) + 1e-6

            cosine = (v1[0]*v2[0] + v1[1]*v2[1]) / (norm1 * norm2)

            # penalize backward motion
            direction_penalty = (1.0 - cosine)  # 0 good, 2 worst
        else:
            direction_penalty = 0.0

        # weighted cost
        return dist_score + 2.0 * direction_penalty

    wp = min(candidates, key=score)
    x, y, yaw = wp

    blackboard["last_node_xy"] = (x, y)
    blackboard["current_node"] = node_key(wp)
    blackboard["pose"] = make_pose(x, y, yaw)
    blackboard["interruptible"] = True
    blackboard["enable_pub"].publish(Bool(data=False))

    blackboard["node"].get_logger().info(
        f"BEST_NODE -> ({x:.2f}, {y:.2f})"
    )

    return HAS_NEXT

def mark_done(blackboard: Blackboard) -> str:
    """
    This function simply marks the achieved node as visited
    """
    key = blackboard.get("current_node")
    if key is not None:
        blackboard["done_nodes"].add(key)
        blackboard["node"].get_logger().info(f"node {key} reached")
    return SUCCEED

def mark_failed(blackboard: Blackboard) -> str:
    """
    This function marks the desired node to failed after multiple retries
    """
    key = blackboard.get("current_node")
    if key is None:
        return SUCCEED
    counts = blackboard["failed_count"]
    counts[key] = counts.get(key, 0) + 1
    cap = blackboard["max_node_retries"]
    if counts[key] >= cap:
        blackboard["done_nodes"].add(key)
        blackboard["node"].get_logger().warn(
            f"node {key} failed {counts[key]}/{cap} — permanently excluded")
    else:
        blackboard["node"].get_logger().warn(
            f"node {key} failed {counts[key]}/{cap} — will retry later")
    return SUCCEED

def maybe_reset(blackboard: Blackboard) -> str:
    """
    This function deals with the case of where the robot explored all the zone but did not catch 
    all duplos
    """
    time_left = blackboard["t_end"] - time.time()
    missing = blackboard["expected_duplos_in_zone"] - blackboard["total_collected"]
    if missing > 0 and time_left > blackboard["reset_min_time_s"]:
        blackboard["node"].get_logger().warn(
            f"GRID EXHAUSTED — {missing} duplos still expected, "
            f"{time_left:.0f}s left → resetting grid for another pass")
        return RESET
    blackboard["node"].get_logger().info(
        f"GRID EXHAUSTED — finishing "
        f"(missing={missing}, time_left={time_left:.0f}s)")
    return DONE

def reset_grid(blackboard: Blackboard) -> str:
    """
    Resets the exploration zone grid
    """
    blackboard["done_nodes"] = set()
    blackboard["failed_count"] = {}
    return SUCCEED

def set_approach_pose(blackboard: Blackboard) -> str:
    """
    Defines the approaching pose for duplo collection
    """
    tx, ty = blackboard["target_xy"]
    rx, ry = blackboard.get("robot_xy", (tx - 0.5, ty))
    yaw = math.atan2(ty - ry, tx - rx)
    standoff = blackboard["approach_standoff"]
    px = tx - standoff * math.cos(yaw)
    py = ty - standoff * math.sin(yaw)
    blackboard["pose"] = make_pose(px, py, yaw)
    blackboard["interruptible"] = False
    return SUCCEED

def collect(blackboard: Blackboard) -> str:
    """
    Physical collection: snapshot the Arduino caught-counter, open-loop push
    forward over the duplo, then snapshot again. The Arduino sensor is the
    only source of truth for "did we actually catch something" — vision is
    used to PLAN the collection, not to confirm it.
    """
    node = blackboard["node"]
    # Defensive: keep duplo_approach disabled, the push owns the wheels via ramp_vel.
    blackboard["enable_pub"].publish(Bool(data=False))

    caught_before = blackboard.get("caught_total", 0)

    twist = Twist()
    twist.linear.x = float(blackboard["extra_push_speed"])
    hz = 20.0
    steps = max(1, round(blackboard["extra_push_time"] * hz))
    node.get_logger().info(
        f"COLLECT: push {blackboard['extra_push_speed']:.2f} m/s "
        f"for {blackboard['extra_push_time']:.1f}s")
    for _ in range(steps):
        blackboard["ramp_pub"].publish(twist)
        time.sleep(1.0 / hz)
    blackboard["ramp_pub"].publish(Twist())

    # let the sensor latch the last-instant detection before sampling
    time.sleep(blackboard.get("sensor_settle_s", 0.5))

    caught_after = blackboard.get("caught_total", 0)
    delta = max(0, caught_after - caught_before)
    blackboard["caught_delta"] = delta
    node.get_logger().info(
        f"COLLECT: sensor delta = {delta}  "
        f"(before={caught_before}, after={caught_after})")
    return SUCCEED


def check_collected(blackboard: Blackboard) -> str:
    """
    Arduino sensor decides. If the catch counter rose, we OWN the map update:
    publish a /duplo_collected point for the nearest <delta> clusters within
    collect_radius so duplo_map prunes exactly what we took (no implicit reliance
    on vision losing track of the cluster). Fallback: if no cluster is near
    enough, publish the target_xy so at least the planned cluster is cleared.
    """
    node = blackboard["node"]
    delta = blackboard.get("caught_delta", 0)
    if delta <= 0:
        node.get_logger().warn("IS_COLLECTED: sensor reported no catch")
        return NOT_COLLECTED

    duplo_map = blackboard.get("duplo_map") or PoseArray()
    rx, ry = blackboard.get("robot_xy", (0.0, 0.0))
    radius = blackboard["collect_radius"]
    nearby = sorted(
        [p for p in duplo_map.poses
         if math.hypot(p.position.x - rx, p.position.y - ry) < radius],
        key=lambda p: math.hypot(p.position.x - rx, p.position.y - ry))[:delta]

    to_publish = nearby if nearby else []
    if not to_publish:
        tx, ty = blackboard["target_xy"]
        node.get_logger().warn(
            f"IS_COLLECTED: sensor saw {delta} catch(es) but no cluster "
            f"within {radius:.2f}m of robot — clearing target instead")
        to_publish = [None]
        for _ in to_publish:
            point = PointStamped()
            point.header.frame_id = "map"
            point.point.x = float(tx)
            point.point.y = float(ty)
            blackboard["collected_pub"].publish(point)
    else:
        for p in to_publish:
            point = PointStamped()
            point.header.frame_id = "map"
            point.point.x = float(p.position.x)
            point.point.y = float(p.position.y)
            blackboard["collected_pub"].publish(point)

    blackboard["carrying"] += delta
    blackboard["total_collected"] += delta
    node.get_logger().info(
        f"COLLECTED {delta}  carrying={blackboard['carrying']}/{blackboard['max_duplo']}  "
        f"total={blackboard['total_collected']}/"
        f"{blackboard['expected_duplos_in_zone']}")
    return COLLECTED


def check_retries(blackboard: Blackboard) -> str:
    blackboard["approach_retries"] = blackboard.get("approach_retries", 0) + 1
    if blackboard["approach_retries"] >= blackboard["max_approach_retries"]:
        blackboard["node"].get_logger().warn(
            f"giving up on target after {blackboard['approach_retries']} retries")
        return GIVE_UP
    blackboard["node"].get_logger().info(
        f"retry {blackboard['approach_retries']}/"
        f"{blackboard['max_approach_retries']}")
    return RETRY


def check_max_duplo(blackboard: Blackboard) -> str:
    if blackboard["carrying"] >= blackboard["max_duplo"]:
        return MAX_REACHED
    return MORE_DUPLO


def set_dropoff_pose(blackboard: Blackboard) -> str:
    dx, dy, dyaw = blackboard["dropoff_pose"]
    blackboard["pose"] = make_pose(dx, dy, dyaw)
    blackboard["interruptible"] = False
    return SUCCEED


def reset_after_dropoff(blackboard: Blackboard) -> str:
    blackboard["carrying"] = 0
    blackboard["node"].get_logger().info("DROP_OFF complete; resuming search")
    return SUCCEED


def create_approach_sm() -> StateMachine:
    sm = StateMachine(outcomes=[COLLECTED, GIVE_UP, MAX_REACHED, CANCEL])

    sm.add_state(
        "SET_APPROACH_POSE",
        CbState([SUCCEED], set_approach_pose),
        transitions={SUCCEED: "NAV_CLOSE"},
    )
    sm.add_state(
        "NAV_CLOSE",
        Nav2State(),
        transitions={
            SUCCEED: "COLLECT",
            ABORT: "CHECK_RETRIES",
            CANCEL: CANCEL,
            DUPLO_FOUND: "COLLECT",
        },
    )
    sm.add_state(
        "COLLECT",
        CbState([SUCCEED], collect),
        transitions={SUCCEED: "IS_COLLECTED"},
    )
    sm.add_state(
        "IS_COLLECTED",
        CbState([COLLECTED, NOT_COLLECTED], check_collected),
        transitions={COLLECTED: "CHECK_MAX_DUPLO", NOT_COLLECTED: "CHECK_RETRIES"},
    )
    sm.add_state(
        "CHECK_RETRIES",
        CbState([RETRY, GIVE_UP], check_retries),
        transitions={RETRY: "NAV_CLOSE", GIVE_UP: GIVE_UP},
    )
    sm.add_state(
        "CHECK_MAX_DUPLO",
        CbState([MAX_REACHED, MORE_DUPLO], check_max_duplo),
        transitions={MAX_REACHED: MAX_REACHED, MORE_DUPLO: COLLECTED},
    )

    return sm


def create_dropoff_sm() -> StateMachine:
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])

    sm.add_state(
        "SET_DROPOFF_POSE",
        CbState([SUCCEED], set_dropoff_pose),
        transitions={SUCCEED: "NAV_TO_DROPOFF"},
    )
    sm.add_state(
        "NAV_TO_DROPOFF",
        Nav2State(),
        transitions={
            SUCCEED: "RESET",
            ABORT: ABORT,
            CANCEL: CANCEL,
            DUPLO_FOUND: "RESET",
        },
    )
    sm.add_state(
        "RESET",
        CbState([SUCCEED], reset_after_dropoff),
        transitions={SUCCEED: SUCCEED},
    )

    return sm


def create_mission_sm() -> StateMachine:
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])

    sm.add_state(
        "INIT_GRID",
        CbState([SUCCEED], init_grid),
        transitions={SUCCEED: "CHECK_MISSION_ALIVE"},
    )
    sm.add_state(
        "CHECK_MISSION_ALIVE",
        CbState([KEEP_GOING, DONE], check_mission_alive),
        transitions={KEEP_GOING: "SELECT_TARGET", DONE: SUCCEED},
    )
    sm.add_state(
        "SELECT_TARGET",
        CbState([FOUND, NONE], select_target),
        transitions={FOUND: "APPROACH_TARGET", NONE: "BEST_NODE"},
    )
    sm.add_state(
        "BEST_NODE",
        CbState([HAS_NEXT, NO_NEXT], best_node),
        transitions={HAS_NEXT: "IS_REACHABLE", NO_NEXT: "MAYBE_RESET"},
    )
    sm.add_state(
        "IS_REACHABLE",
        IsReachableState(),
        transitions={
            REACHABLE: "NAVIGATE",
            UNREACHABLE: "MARK_FAILED",
            ABORT: "MARK_FAILED",
            CANCEL: CANCEL,
        },
    )
    sm.add_state(
        "NAVIGATE",
        Nav2State(),
        transitions={
            SUCCEED: "MARK_DONE",
            ABORT: "MARK_FAILED",
            CANCEL: CANCEL,
            DUPLO_FOUND: "SELECT_TARGET",
        },
    )
    sm.add_state(
        "MARK_DONE",
        CbState([SUCCEED], mark_done),
        transitions={SUCCEED: "CHECK_MISSION_ALIVE"},
    )
    sm.add_state(
        "MARK_FAILED",
        CbState([SUCCEED], mark_failed),
        transitions={SUCCEED: "CHECK_MISSION_ALIVE"},
    )
    sm.add_state(
        "MAYBE_RESET",
        CbState([RESET, DONE], maybe_reset),
        transitions={RESET: "RESET_GRID", DONE: SUCCEED},
    )
    sm.add_state(
        "RESET_GRID",
        CbState([SUCCEED], reset_grid),
        transitions={SUCCEED: "CHECK_MISSION_ALIVE"},
    )
    sm.add_state(
        "APPROACH_TARGET",
        create_approach_sm(),
        transitions={
            COLLECTED: "SELECT_TARGET",
            MAX_REACHED: "DROP_OFF_DUPLO",
            GIVE_UP: "SELECT_TARGET",
            CANCEL: CANCEL,
        },
    )
    sm.add_state(
        "DROP_OFF_DUPLO",
        create_dropoff_sm(),
        transitions={
            SUCCEED: "CHECK_MISSION_ALIVE",
            ABORT: "CHECK_MISSION_ALIVE",
            CANCEL: CANCEL,
        },
    )

    return sm


def main() -> None:
    rclpy.init()
    set_ros_loggers()

    node = rclpy.create_node("explore_zone")
    node.declare_parameter("waypoints_file", "/maps/arena/waypoints_right.yaml")
    node.declare_parameter("mission_time", 300.0)
    node.declare_parameter("max_duplo", 4)
    node.declare_parameter("expected_duplos_in_zone", 12)
    node.declare_parameter("dropoff_x", 0.35)
    node.declare_parameter("dropoff_y", 0.30)
    node.declare_parameter("dropoff_yaw", -1.57)
    node.declare_parameter("nav_timeout_s", 60.0)
    node.declare_parameter("nav_close_timeout_s", 30.0)
    node.declare_parameter("plan_timeout_s", 10.0)
    node.declare_parameter("max_node_retries", 2)
    node.declare_parameter("duplo_match_tol", 0.20)
    node.declare_parameter("reset_min_time_s", 30.0)
    node.declare_parameter("caught_topic", "/duplos_caught_total")
    node.declare_parameter("extra_push_time", 2.0)
    node.declare_parameter("extra_push_speed", 0.10)
    node.declare_parameter("sensor_settle_s", 0.5)
    node.declare_parameter("collect_radius", 0.30)

    blackboard = Blackboard()
    blackboard["node"] = node
    blackboard["vel_pub"] = node.create_publisher(Twist, "duplo_vel", 10)
    blackboard["ramp_pub"] = node.create_publisher(Twist, "ramp_vel", 10)
    blackboard["enable_pub"] = node.create_publisher(Bool, "/enable_duplo_collection", 10)
    blackboard["collected_pub"] = node.create_publisher(PointStamped, "/duplo_collected", 10)
    blackboard["robot_xy"] = (0.0, 0.0)
    blackboard["duplo_map"] = None
    blackboard["interruptible"] = True
    blackboard["caught_total"] = 0
    blackboard["caught_delta"] = 0
    blackboard["waypoints_file"] = node.get_parameter("waypoints_file").value
    blackboard["mission_time"] = float(node.get_parameter("mission_time").value)
    blackboard["max_duplo"] = int(node.get_parameter("max_duplo").value)
    blackboard["expected_duplos_in_zone"] = int(
        node.get_parameter("expected_duplos_in_zone").value)
    blackboard["dropoff_pose"] = (
        float(node.get_parameter("dropoff_x").value),
        float(node.get_parameter("dropoff_y").value),
        float(node.get_parameter("dropoff_yaw").value),
    )
    blackboard["approach_standoff"] = 0.4
    blackboard["collect_radius"] = float(node.get_parameter("collect_radius").value)
    blackboard["extra_push_time"] = float(node.get_parameter("extra_push_time").value)
    blackboard["extra_push_speed"] = float(node.get_parameter("extra_push_speed").value)
    blackboard["sensor_settle_s"] = float(node.get_parameter("sensor_settle_s").value)
    blackboard["max_approach_retries"] = 3
    blackboard["nav_timeout_s"] = float(node.get_parameter("nav_timeout_s").value)
    blackboard["nav_close_timeout_s"] = float(
        node.get_parameter("nav_close_timeout_s").value)
    blackboard["plan_timeout_s"] = float(node.get_parameter("plan_timeout_s").value)
    blackboard["max_node_retries"] = int(node.get_parameter("max_node_retries").value)
    blackboard["duplo_match_tol"] = float(node.get_parameter("duplo_match_tol").value)
    blackboard["reset_min_time_s"] = float(node.get_parameter("reset_min_time_s").value)

    node.create_subscription(PoseArray, "/duplo_map", lambda m: on_duplo_map(m, blackboard), 10)
    node.create_subscription(
        PoseWithCovarianceStamped, "/amcl_pose", lambda m: on_amcl_pose(m, blackboard), 10)
    node.create_subscription(
        Int32, node.get_parameter("caught_topic").value,
        lambda m: on_caught_total(m, blackboard), 10)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    sm = create_mission_sm()
    YasminViewerPub("EXPLORE_ZONE", sm)

    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()
    finally:
        blackboard["vel_pub"].publish(Twist())
        blackboard["ramp_pub"].publish(Twist())
        blackboard["enable_pub"].publish(Bool(data=False))
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
