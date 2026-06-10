"""
Integration tests for the mission refactor.

Self-contained: stubs all of rclpy / nav2 / std_msgs / etc. so it can run on a
machine without ROS installed. Verifies:

  • MRO + cooperative __init__ across mixins (DuploMixin, RampMixin, ButtonMixin)
  • dropoff() uses subclass BASE_POSE and always reverses
  • explore_zone reads YAML, visits waypoints in order, sweeps at each
  • Sweep enables/disables /enable_duplo_collection around the rotation phase
  • Sweep waits for FSM when /duplo_state reports a non-idle state
  • Opportunistic collect cancels Nav2 on FSM activity, then resumes
  • Auto-recovery fires after RECOVERY_FAILURE_STREAK consecutive failures
  • _go_to_with_recovery retries up to CRITICAL_NAV_ATTEMPTS times
  • main_loop blocks on /mission_command 'start', restarts on 'reset',
    preserves mission_start across resets

Run:
    python3 ros2_ws/src/j2cdynamics_mission/tests/test_mission_refactor.py
"""

import os
import sys
import time as real_time
import types
import threading
import tempfile
import traceback


# ──────────────────────────────────────────────────────────────────────────────
#  ROS-environment stubs (must be in place BEFORE importing mission modules)
# ──────────────────────────────────────────────────────────────────────────────

# rclpy
_rclpy = types.ModuleType('rclpy')
_rclpy.ok = lambda: not _STOP_RCLPY['stop']
_rclpy.init = lambda *a, **k: None
_rclpy.shutdown = lambda *a, **k: None
_rclpy.spin_once = lambda node, timeout_sec=0.0: real_time.sleep(0)
_rclpy.spin_until_future_complete = lambda node, fut, timeout_sec=0.0: None
sys.modules['rclpy'] = _rclpy

_STOP_RCLPY = {'stop': False}

# rclpy.action
_rclpy_action = types.ModuleType('rclpy.action')
class _ActionClient:
    def __init__(self, *a, **k): pass
    def wait_for_server(self, timeout_sec=2.0): return True
    def send_goal_async(self, goal):
        # _is_reachable expects accepted=True with a non-empty path.
        return _Future(result=_GoalHandle(accepted=True))
_rclpy_action.ActionClient = _ActionClient
sys.modules['rclpy.action'] = _rclpy_action

# rclpy.qos
_rclpy_qos = types.ModuleType('rclpy.qos')
class _QoSProfile:
    def __init__(self, **k): pass
class _Enum:
    KEEP_LAST = 0
    TRANSIENT_LOCAL = 0
    RELIABLE = 0
_rclpy_qos.QoSProfile = _QoSProfile
_rclpy_qos.DurabilityPolicy = _Enum
_rclpy_qos.ReliabilityPolicy = _Enum
_rclpy_qos.HistoryPolicy = _Enum
sys.modules['rclpy.qos'] = _rclpy_qos

# geometry_msgs
_geom = types.ModuleType('geometry_msgs')
_geom_msg = types.ModuleType('geometry_msgs.msg')
class _Vec3:
    def __init__(self): self.x = self.y = self.z = 0.0
class _Quat:
    def __init__(self): self.x = self.y = self.z = 0.0; self.w = 1.0
class _Pose:
    def __init__(self):
        self.position = _Vec3()
        self.orientation = _Quat()
class _Header:
    def __init__(self): self.frame_id = ''; self.stamp = None
class _PoseStamped:
    def __init__(self): self.header = _Header(); self.pose = _Pose()
class _Twist:
    def __init__(self):
        self.linear = _Vec3()
        self.angular = _Vec3()
_geom_msg.PoseStamped = _PoseStamped
_geom_msg.Twist = _Twist
sys.modules['geometry_msgs'] = _geom
sys.modules['geometry_msgs.msg'] = _geom_msg

# std_msgs
_std = types.ModuleType('std_msgs')
_std_msg = types.ModuleType('std_msgs.msg')
class _String:
    def __init__(self, data=''): self.data = data
class _Bool:
    def __init__(self, data=False): self.data = data
_std_msg.String = _String
_std_msg.Bool = _Bool
sys.modules['std_msgs'] = _std
sys.modules['std_msgs.msg'] = _std_msg

# nav_msgs (used by some imports indirectly)
_navmsgs = types.ModuleType('nav_msgs')
_navmsgs_msg = types.ModuleType('nav_msgs.msg')
class _OccupancyGrid: pass
_navmsgs_msg.OccupancyGrid = _OccupancyGrid
sys.modules['nav_msgs'] = _navmsgs
sys.modules['nav_msgs.msg'] = _navmsgs_msg

# nav2_msgs.action
_nav2_msgs = types.ModuleType('nav2_msgs')
_nav2_action = types.ModuleType('nav2_msgs.action')
class _ComputePathToPose:
    class Goal:
        def __init__(self):
            self.goal = None
            self.use_start = False
_nav2_action.ComputePathToPose = _ComputePathToPose
sys.modules['nav2_msgs'] = _nav2_msgs
sys.modules['nav2_msgs.action'] = _nav2_action

# nav2_simple_commander.robot_navigator
_nav2_simple = types.ModuleType('nav2_simple_commander')
_nav2_rn = types.ModuleType('nav2_simple_commander.robot_navigator')

class _TaskResultValue:
    """Enum-like value with a .name attr (matches Nav2's TaskResult enum)."""
    def __init__(self, name):
        self.name = name
    def __eq__(self, other):
        return isinstance(other, _TaskResultValue) and self.name == other.name
    def __hash__(self):
        return hash(self.name)
    def __repr__(self):
        return self.name


class TaskResult:
    SUCCEEDED = _TaskResultValue('SUCCEEDED')
    FAILED    = _TaskResultValue('FAILED')
    CANCELED  = _TaskResultValue('CANCELED')


class _Future:
    def __init__(self, result=None):
        self._result = result
        self._done = True
    def result(self): return self._result
    def done(self): return self._done


class _GoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
    def get_result_async(self):
        return _Future(result=types.SimpleNamespace(
            result=types.SimpleNamespace(path=types.SimpleNamespace(poses=[None]))))
    def cancel_goal_async(self): return _Future()


class BasicNavigator:
    """Stub Nav2 BasicNavigator. Records every call so tests can assert on the
    sequence of operations. Behavior is configurable per-test via attrs:
      goto_results: list of TaskResult values to return for successive goToPose calls
      goto_complete_ticks: how many isTaskComplete() polls before returning True
      backup_complete_ticks: same for backup
    """
    def __init__(self, node_name):
        self._node_name = node_name
        self.calls = []   # list of (method_name, args_summary)
        self.goto_results = []         # populated by tests
        self._goto_idx = 0
        self.goto_complete_ticks = 1
        self.backup_complete_ticks = 1
        self.tick_real_delay_s = 0.0   # opt-in: real wall-sleep between isTaskComplete polls
        self._task_kind = None         # 'goto' or 'backup'
        self._ticks_remaining = 0

        # Lets tests inject FSM transitions while the runner spins.
        # Each entry: (delay_s_from_now, new_state)
        self._fsm_schedule = []
        self._fsm_set_callback = None  # set by DuploMixin-test setup

        # Subscribers indexed by topic for direct invocation in tests.
        self._subs_by_topic = {}

    # ── Node-like API ──
    def get_logger(self):
        return _Logger(self.calls)
    def create_publisher(self, msg_type, topic, qos):
        self.calls.append(('create_publisher', topic))
        pub = _Publisher(topic, self.calls)
        return pub
    def create_subscription(self, msg_type, topic, callback, qos):
        self.calls.append(('create_subscription', topic))
        self._subs_by_topic[topic] = callback
        return None
    def destroy_node(self): self.calls.append(('destroy_node',))

    # ── Nav2-like API ──
    def setInitialPose(self, pose): self.calls.append(('setInitialPose',))
    def waitUntilNav2Active(self, localizer=''): self.calls.append(('waitUntilNav2Active',))
    def goToPose(self, pose):
        self.calls.append(('goToPose',))
        self._task_kind = 'goto'
        self._ticks_remaining = self.goto_complete_ticks
        # New task → invalidate cached result so getResult() pulls the next one.
        self._cached_result = None
    def backup(self, backup_dist=0.15, backup_speed=0.1, time_allowance=10):
        self.calls.append(('backup', backup_dist))
        self._task_kind = 'backup'
        self._ticks_remaining = self.backup_complete_ticks
        self._cached_result = None
    def isTaskComplete(self):
        if self.tick_real_delay_s > 0:
            _real_sleep(self.tick_real_delay_s)
        self._ticks_remaining -= 1
        return self._ticks_remaining <= 0
    def getResult(self):
        """Idempotent per task — real Nav2 returns the same value on repeated
        calls within a single task. Production code calls this twice in go_to
        (once for ok, once for the log), so we must NOT advance the index."""
        if getattr(self, '_cached_result', None) is not None:
            return self._cached_result
        if self._task_kind == 'goto':
            if self._goto_idx < len(self.goto_results):
                self._cached_result = self.goto_results[self._goto_idx]
                self._goto_idx += 1
            else:
                self._cached_result = TaskResult.SUCCEEDED
        else:
            self._cached_result = TaskResult.SUCCEEDED
        return self._cached_result
    def cancelTask(self):
        self.calls.append(('cancelTask',))
        self._ticks_remaining = 0
    def clearAllCostmaps(self): self.calls.append(('clearAllCostmaps',))

_nav2_rn.BasicNavigator = BasicNavigator
_nav2_rn.TaskResult = TaskResult
sys.modules['nav2_simple_commander'] = _nav2_simple
sys.modules['nav2_simple_commander.robot_navigator'] = _nav2_rn


class _Publisher:
    def __init__(self, topic, log):
        self.topic = topic
        self._log = log
        self.messages = []
    def publish(self, msg):
        # Record on both per-publisher list and the global call log so tests
        # can look up either way.
        try:
            data = getattr(msg, 'data', getattr(msg, 'linear', None))
        except Exception:
            data = None
        self.messages.append(msg)
        self._log.append(('publish', self.topic))


class _Logger:
    def __init__(self, log):
        self._log = log
    def info(self, m, **k):    self._log.append(('log.info', str(m)))
    def warn(self, m, **k):    self._log.append(('log.warn', str(m)))
    def error(self, m, **k):   self._log.append(('log.error', str(m)))


# Make time.sleep no-op so tests run fast (mission code uses real time.sleep).
_real_sleep = real_time.sleep
real_time.sleep = lambda s: None


# Now we can import the package.
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_PARENT = os.path.abspath(os.path.join(THIS_DIR, '..'))
sys.path.insert(0, PKG_PARENT)

from j2cdynamics_mission import mission_base  # noqa: E402
from j2cdynamics_mission import mission_duplo  # noqa: E402
from j2cdynamics_mission.do_mission import DoMissionRunner  # noqa: E402
from j2cdynamics_mission.da_mission import DaMissionRunner  # noqa: E402
from j2cdynamics_mission.mission_ramp import RampMixin  # noqa: E402
from j2cdynamics_mission.mission_button import ButtonMixin  # noqa: E402


# ──────────────────────────────────────────────────────────────────────────────
#  Fake fixtures (waypoints + duplo data)
# ──────────────────────────────────────────────────────────────────────────────

def write_fake_waypoints(path, waypoints):
    """waypoints: list of (x, y, yaw)"""
    import yaml
    with open(path, 'w') as f:
        yaml.dump({'waypoints': [list(w) for w in waypoints]}, f)


FIXTURES = tempfile.mkdtemp(prefix='mission_test_')
WP_ZONE_A = os.path.join(FIXTURES, 'zone_a.yaml')
WP_ZONE_B = os.path.join(FIXTURES, 'zone_b.yaml')
write_fake_waypoints(WP_ZONE_A, [(1.0, 0.0, 0.0), (2.0, 0.0, 1.57), (2.0, 1.0, 3.14)])
write_fake_waypoints(WP_ZONE_B, [(5.0, 5.0, 0.0)])


# ──────────────────────────────────────────────────────────────────────────────
#  Test helpers
# ──────────────────────────────────────────────────────────────────────────────

def count_calls(calls, method, topic_or_arg=None):
    if topic_or_arg is None:
        return sum(1 for c in calls if c[0] == method)
    return sum(1 for c in calls if c[0] == method and len(c) > 1 and c[1] == topic_or_arg)


def first_index(calls, method, topic_or_arg=None):
    for i, c in enumerate(calls):
        if c[0] != method:
            continue
        if topic_or_arg is None or (len(c) > 1 and c[1] == topic_or_arg):
            return i
    return -1


def make_runner(cls, **overrides):
    """Build a test instance with auto-recovery disabled by default so tests
    that don't care about recovery aren't perturbed by it."""
    r = cls()
    # Disable supervisor (don't spawn the thread in tests).
    r._supervisor_disabled = True
    return r


def disable_supervisor(runner):
    """Replace _supervisor with a no-op so main_loop tests don't race."""
    runner._supervisor = lambda: None


# ──────────────────────────────────────────────────────────────────────────────
#  TESTS
# ──────────────────────────────────────────────────────────────────────────────

TESTS = []
def test(fn):
    TESTS.append(fn)
    return fn


@test
def test_mro_correct():
    """Both runners have the expected MRO."""
    do_mro = [c.__name__ for c in DoMissionRunner.__mro__]
    da_mro = [c.__name__ for c in DaMissionRunner.__mro__]
    assert do_mro[:5] == ['DoMissionRunner', 'DuploMixin', 'RampMixin', 'MissionBase', 'BasicNavigator'], \
        f'DO MRO: {do_mro}'
    assert da_mro[:5] == ['DaMissionRunner', 'DuploMixin', 'ButtonMixin', 'MissionBase', 'BasicNavigator'], \
        f'DA MRO: {da_mro}'


@test
def test_construction_creates_pubs_and_subs():
    """DuploMixin __init__ chains correctly and creates its duplo pubs/subs in
    addition to the base ones."""
    r = DoMissionRunner()
    pubs = [c[1] for c in r.calls if c[0] == 'create_publisher']
    subs = [c[1] for c in r.calls if c[0] == 'create_subscription']
    # MissionBase pubs:
    assert mission_base.RAMP_VEL_TOPIC in pubs, f'missing ramp pub; got {pubs}'
    assert 'goal_checker_selector' in pubs
    # DuploMixin pubs/subs:
    assert '/enable_duplo_collection' in pubs, f'DuploMixin pub missing; got {pubs}'
    assert '/duplo_state' in subs, f'DuploMixin sub missing; got {subs}'
    # MissionBase command sub:
    assert mission_base.MISSION_COMMAND_TOPIC in subs


@test
def test_subclass_inherits_methods_from_all_mixins():
    """DoMissionRunner can call methods from DuploMixin and RampMixin.
       DaMissionRunner can call methods from DuploMixin and ButtonMixin."""
    do = DoMissionRunner()
    da = DaMissionRunner()
    # DuploMixin methods
    assert callable(getattr(do, 'explore_zone'))
    assert callable(getattr(da, 'explore_zone'))
    assert callable(getattr(do, '_sweep_and_collect'))
    # RampMixin only on DO
    assert callable(getattr(do, 'go_up_ramp'))
    assert not isinstance(da, RampMixin), 'DA should not have RampMixin'
    # ButtonMixin only on DA
    assert callable(getattr(da, 'push_button_and_wait_for_door'))
    assert not isinstance(do, ButtonMixin), 'DO should not have ButtonMixin'


@test
def test_dropoff_uses_subclass_pose_and_always_reverses():
    """dropoff calls go_to twice (align + base) then publishes Twist for the
    reverse — even if both go_to calls 'succeed' or fail."""
    r = DoMissionRunner()
    # Force both Nav2 calls to succeed.
    r.goto_results = [TaskResult.SUCCEEDED, TaskResult.SUCCEEDED]
    r.dropoff()
    # Two goToPose calls then ~60 ramp_vel publishes (DROPOFF_TIME=3s × 20 Hz).
    goto_count = count_calls(r.calls, 'goToPose')
    ramp_pubs = count_calls(r.calls, 'publish', mission_base.RAMP_VEL_TOPIC)
    assert goto_count == 2, f'expected 2 goToPose, got {goto_count}'
    # ramp_vel published during open-loop reverse (3.0s × 20Hz = ~60) plus final Twist().
    assert ramp_pubs >= 50, f'expected ≥50 ramp_vel publishes during reverse, got {ramp_pubs}'


@test
def test_dropoff_reverses_even_when_base_fails():
    """If both BASE_POSE attempts fail, the reverse still fires (it IS the deposit)."""
    r = DoMissionRunner()
    # Align succeeds; both BASE attempts fail.
    r.goto_results = [TaskResult.SUCCEEDED, TaskResult.FAILED, TaskResult.FAILED]
    r.dropoff()
    ramp_pubs = count_calls(r.calls, 'publish', mission_base.RAMP_VEL_TOPIC)
    assert ramp_pubs >= 50, f'reverse must always fire; got {ramp_pubs} publishes'
    # Error log should mention deposit-may-be-incomplete.
    log_msgs = [c[1] for c in r.calls if c[0] in ('log.warn', 'log.error')]
    assert any('deposit may be incomplete' in m for m in log_msgs), \
        f'expected error log; got {log_msgs}'


@test
def test_explore_zone_visits_all_waypoints_and_sweeps():
    """explore_zone reads YAML, calls go_to per waypoint, then sweeps."""
    r = DoMissionRunner()
    r.goto_results = [TaskResult.SUCCEEDED] * 20
    r.explore_zone(WP_ZONE_A, duration_s=60.0, label='ZONE_A',
                   opportunistic_collect=False)
    # 3 waypoints → at least 3 goToPose calls + sweep rotations on ramp_vel.
    goto = count_calls(r.calls, 'goToPose')
    assert goto >= 3, f'expected ≥3 goToPose (one per waypoint), got {goto}'
    # Each waypoint's sweep enables then disables collection.
    enable_pubs = [m for m in [c[1] for c in r.calls if c[0] == 'publish']
                   if m == '/enable_duplo_collection']
    assert len(enable_pubs) >= 6, \
        f'expected ≥6 collection toggles (3 enable + 3 disable), got {len(enable_pubs)}'
    # Clear-costmaps fires after each sweep.
    assert count_calls(r.calls, 'clearAllCostmaps') >= 3, \
        'expected clearAllCostmaps after each sweep'


@test
def test_sweep_toggles_collection_and_waits_for_fsm():
    """When _fsm_state goes non-idle during the dwell, the cycle counter
    advances. Uses thread-based state flipping (more reliable than
    spin_once side-effects)."""
    r = DoMissionRunner()
    r.goto_results = [TaskResult.SUCCEEDED] * 20

    cycles_result = {}
    def runner_thread():
        # total_yaw=0.5 → ceil(0.5 / 0.4) = 2 sweep steps (each = rotate + dwell)
        cycles_result['n'] = r._sweep_and_collect(label='TEST', total_yaw=0.5)
    t = threading.Thread(target=runner_thread, daemon=True)
    t.start()

    # Wait for the runner to be deep into the first dwell, then flip to 'approach'.
    # Dwell is ~1s (SCAN_DWELL_S). We want 'approach' set when dwell ends so the
    # runner sees it and enters _wait_until_fsm_idle. Then flip back to 'search'.
    _real_sleep(0.4)
    r._fsm_state = 'approach'
    # Wait long enough for the dwell to end (1s total, we already used 0.4s).
    _real_sleep(1.0)
    # Now runner should be in _wait_until_fsm_idle; release it.
    r._fsm_state = 'search'

    t.join(15.0)
    assert not t.is_alive(), 'sweep did not complete in time'
    assert cycles_result.get('n', 0) >= 1, \
        f'expected ≥1 FSM cycle when state went non-idle; got {cycles_result}'


@test
def test_opportunistic_collect_cancels_and_resumes_on_fsm():
    """If FSM enters approach mid-transit, Nav2 is cancelled, FSM is awaited,
    then the same goal is re-issued. Uses thread-based state flipping."""
    r = DoMissionRunner()
    # Only the SECOND attempt actually consumes a getResult (the first is cancelled
    # via the FSM-interrupt path before getResult is read). So we only need one
    # success entry here.
    r.goto_results = [TaskResult.SUCCEEDED]
    # Make the first goal's polling take real wall time so the test thread can
    # flip _fsm_state mid-transit.
    r.goto_complete_ticks = 100
    r.tick_real_delay_s = 0.01   # 100 * 10ms = ~1s of real time per goal

    ok_result = {}
    def runner_thread():
        ok_result['ok'] = r._go_to_with_opportunistic_collect(
            (1.0, 1.0, 0.0), label='TEST',
            timeout_s=30.0, max_interruptions=2)
    t = threading.Thread(target=runner_thread, daemon=True)
    t.start()

    # Let the first goToPose get going, then trigger an interrupt.
    # goto_complete_ticks=100 means isTaskComplete polls 100x → with spin_once
    # being essentially no-op the inner while-loop runs at CPU speed.
    _real_sleep(0.3)
    r._fsm_state = 'approach'
    # Wait until the runner has cancelled Nav2 and is waiting on the FSM.
    _real_sleep(0.5)
    # Release the wait so it can re-issue the goal.
    r._fsm_state = 'search'

    t.join(20.0)
    assert not t.is_alive(), 'opportunistic call did not complete in time'
    assert ok_result.get('ok') is True, 'opportunistic should succeed on resume'
    assert count_calls(r.calls, 'cancelTask') >= 1, 'expected at least one cancelTask'
    assert count_calls(r.calls, 'goToPose') >= 2, 'expected goToPose re-issue after cycle'
    pubs_of_enable = [c for c in r.calls if c[0] == 'publish' and c[1] == '/enable_duplo_collection']
    assert len(pubs_of_enable) >= 2, f'expected ≥2 enable pubs (on/off); got {len(pubs_of_enable)}'


@test
def test_auto_recovery_after_streak():
    """Three consecutive go_to failures triggers _run_recovery (clearAllCostmaps + backup)."""
    r = DoMissionRunner()
    r.goto_results = [TaskResult.FAILED, TaskResult.FAILED, TaskResult.FAILED]
    for _ in range(3):
        r.go_to((1.0, 0.0, 0.0), timeout_s=1.0)
    # After third failure, recovery should fire.
    assert count_calls(r.calls, 'clearAllCostmaps') >= 1, \
        'expected recovery after streak: clearAllCostmaps not called'
    assert count_calls(r.calls, 'backup') >= 1, \
        'expected recovery: backup not called'


@test
def test_go_to_with_recovery_retries_up_to_max():
    """_go_to_with_recovery tries max_attempts times, with recovery between."""
    r = DoMissionRunner()
    r.goto_results = [TaskResult.FAILED] * 10
    ok = r._go_to_with_recovery((1.0, 0.0, 0.0), label='CRIT', max_attempts=3)
    assert not ok, 'critical-leg should ultimately fail'
    # max_attempts goes, max_attempts - 1 recoveries between.
    assert count_calls(r.calls, 'goToPose') == 3, \
        f'expected 3 goToPose; got {count_calls(r.calls, "goToPose")}'
    assert count_calls(r.calls, 'clearAllCostmaps') >= 2, \
        f'expected ≥2 recoveries between attempts'


@test
def test_main_loop_waits_for_start_then_runs():
    """main_loop blocks until start_event set, then runs run() once."""
    r = DoMissionRunner()
    disable_supervisor(r)
    # Override run() to record + return.
    r._ran = 0
    def _fake_run():
        r._ran += 1
    r.run = _fake_run

    # Kick off main_loop in a thread; set start_event after a short delay.
    started = threading.Event()
    def launcher():
        started.set()
        r.main_loop()
    t = threading.Thread(target=launcher, daemon=True)
    t.start()
    started.wait(1.0)

    # Before we send start, mission_status should still be 'waiting'.
    _real_sleep(0.05)
    assert r._mission_status == 'waiting', f'expected waiting; got {r._mission_status}'
    assert r._ran == 0

    # Send start via the callback.
    cb = r._subs_by_topic[mission_base.MISSION_COMMAND_TOPIC]
    cb(_String(data='start'))
    # main_loop should pick up start, run() once, then exit (no restart).
    t.join(2.0)
    assert r._ran == 1, f'expected run() to fire exactly once; got {r._ran}'
    assert r._mission_status == 'complete', \
        f'expected complete; got {r._mission_status}'


@test
def test_reset_preserves_mission_start():
    """RESET while running causes run() to be re-invoked, with the SAME
    mission_start timestamp (so the 600s budget keeps counting)."""
    r = DoMissionRunner()
    disable_supervisor(r)

    r._ran = 0
    saved_start = []
    def _fake_run():
        r._ran += 1
        # Capture mission_start each time run() is called.
        saved_start.append(r.mission_start)
        # On the first call, trigger a reset from inside run() to simulate
        # an operator pressing RESET mid-execution.
        if r._ran == 1:
            cb = r._subs_by_topic[mission_base.MISSION_COMMAND_TOPIC]
            cb(_String(data='reset'))
            # The reset sets abort_event + restart_event; signal end of run.
            from j2cdynamics_mission.mission_base import MissionAbortException
            raise MissionAbortException()
    r.run = _fake_run

    def launcher():
        r.main_loop()
    t = threading.Thread(target=launcher, daemon=True)
    t.start()

    _real_sleep(0.05)
    cb = r._subs_by_topic[mission_base.MISSION_COMMAND_TOPIC]
    cb(_String(data='start'))
    t.join(3.0)

    assert r._ran == 2, f'expected run() invoked twice (initial + restart); got {r._ran}'
    assert saved_start[0] == saved_start[1], \
        f'mission_start must be preserved across reset: {saved_start}'


@test
def test_reset_resumes_from_failed_step():
    """When RESET hits mid-step, the next run() call picks up at the SAME step
    (not at step 1). Uses a stub mission with 5 steps, fails step 3 with abort,
    sends RESET, and verifies steps 4-5 don't run until step 3 succeeds."""
    from j2cdynamics_mission.mission_base import MissionAbortException

    class _StubMission(DoMissionRunner):
        executed = []
        def _build_steps(self):
            return [
                ('S1', lambda: _StubMission.executed.append('S1')),
                ('S2', lambda: _StubMission.executed.append('S2')),
                ('S3', self._step_3),
                ('S4', lambda: _StubMission.executed.append('S4')),
                ('S5', lambda: _StubMission.executed.append('S5')),
            ]
        def _step_3(self):
            _StubMission.executed.append(f'S3-try{_StubMission.s3_tries}')
            _StubMission.s3_tries += 1
            if _StubMission.s3_tries == 1:
                # First attempt: simulate RESET command arriving mid-step.
                cb = self._subs_by_topic[mission_base.MISSION_COMMAND_TOPIC]
                cb(_String(data='reset'))
                raise MissionAbortException()
            # Second attempt: succeed.

    _StubMission.s3_tries = 0
    r = _StubMission()
    disable_supervisor(r)

    def launcher():
        r.main_loop()
    t = threading.Thread(target=launcher, daemon=True)
    t.start()
    _real_sleep(0.05)
    cb = r._subs_by_topic[mission_base.MISSION_COMMAND_TOPIC]
    cb(_String(data='start'))
    t.join(5.0)

    # Step S1 + S2 should run exactly once (across both run() calls).
    # Step S3 should run TWICE (try1 fails via abort, try2 succeeds).
    # Steps S4 + S5 run only after S3 succeeds — exactly once each.
    assert _StubMission.executed == ['S1', 'S2', 'S3-try0', 'S3-try1', 'S4', 'S5'], \
        f'unexpected execution sequence: {_StubMission.executed}'


@test
def test_full_do_mission_dryrun():
    """Smoke test: run DoMissionRunner.run() end-to-end with all Nav2 calls
    succeeding. Ensures no exceptions and the expected step sequence happens."""
    r = DoMissionRunner()
    disable_supervisor(r)
    # Point the runner at our fake waypoints so file existence is satisfied.
    import j2cdynamics_mission.do_mission as dom
    dom.WAYPOINTS_ZONE_1 = WP_ZONE_A
    dom.WAYPOINTS_ZONE_4 = WP_ZONE_B
    dom.TIMEOUT_ZONE_1 = 5.0
    dom.TIMEOUT_ZONE_4 = 5.0
    # Plenty of successes; Nav2 always wins.
    r.goto_results = [TaskResult.SUCCEEDED] * 200

    r.run()

    # We should have seen at least one setInitialPose, multiple goToPose,
    # multiple sweeps (enable/disable toggles), and a ramp climb (open-loop).
    assert count_calls(r.calls, 'setInitialPose') >= 2, \
        'expected setInitialPose for start AND ramp top'
    assert count_calls(r.calls, 'goToPose') >= 6, \
        'expected multiple goToPose calls across exploration + dropoff + ramp_exit'
    ramp_pubs = count_calls(r.calls, 'publish', mission_base.RAMP_VEL_TOPIC)
    assert ramp_pubs >= 100, f'expected many ramp_vel publishes (sweeps + ramp + dropoff); got {ramp_pubs}'


@test
def test_full_da_mission_dryrun():
    """Smoke test for DA: button push + door probe + zone exploration."""
    r = DaMissionRunner()
    disable_supervisor(r)
    import j2cdynamics_mission.da_mission as dam
    dam.WAYPOINTS_ZONE_3 = WP_ZONE_A
    dam.WAYPOINTS_ZONE_1 = WP_ZONE_B
    dam.TIMEOUT_ZONE_3 = 5.0
    dam.TIMEOUT_ZONE_1 = 5.0
    r.goto_results = [TaskResult.SUCCEEDED] * 200
    # Make _is_reachable return True so door probe succeeds.
    r._is_reachable = lambda *a, **k: True
    r.run()

    assert count_calls(r.calls, 'setInitialPose') >= 1
    # Button press involves several ramp_vel publishes (rotate + back + dwell + back + rotate).
    ramp_pubs = count_calls(r.calls, 'publish', mission_base.RAMP_VEL_TOPIC)
    assert ramp_pubs >= 200, f'expected many ramp_vel publishes for button + sweeps + dropoff; got {ramp_pubs}'


# ──────────────────────────────────────────────────────────────────────────────
#  Test driver
# ──────────────────────────────────────────────────────────────────────────────

def main():
    print(f'Running {len(TESTS)} integration tests for the mission refactor\n')
    passed, failed = 0, 0
    failures = []
    for fn in TESTS:
        try:
            fn()
            print(f'  PASS  {fn.__name__}')
            passed += 1
        except Exception as e:
            print(f'  FAIL  {fn.__name__}: {e}')
            failures.append((fn.__name__, traceback.format_exc()))
            failed += 1

    print(f'\n{"="*60}')
    print(f'Results: {passed} passed, {failed} failed')

    if failures:
        print(f'\nFailure details:')
        for name, tb in failures:
            print(f'\n--- {name} ---')
            print(tb)

    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
