"""
mission_button — push the arena button and probe the door for opening.

Used by missions that need to traverse a door behind a button (currently
da_mission.py — Duplo-Aspiration). Stateless mixin: no __init__, just methods
+ constants. Mix in after DuploMixin, before MissionBase:

    class DaMissionRunner(DuploMixin, ButtonMixin, MissionBase): ...

The pattern is open-loop because the button is hit with the robot's rear
bumper (no closed-loop way to verify contact). We push, dwell, retreat, then
ask Nav2's planner whether DOOR_PROBE_POSE is reachable. If yes, the door has
opened; if no, retry with adjusted alignment.

The caller is responsible for getting the robot to BUTTON_APPROACH first
(typically via _go_to_with_recovery for the heading-critical approach).

Tuning knobs are at the top — adjust to the specific button + door geometry.
"""

import math
import time


# ── Button + door constants (tune on hardware) ───────────────────────────────
BUTTON_PUSH_SPEED    = 0.20    # m/s, open-loop forward / backward
BUTTON_BACKOFF_TIME  = 2.0     # s — base backoff time, grows per retry
BUTTON_BACKOFF_STEP  = 0.2     # s added per retry
BUTTON_ROTATE_SPEED  = 1.0     # rad/s for the 180° spin
BUTTON_ROTATE_TIME   = math.pi / BUTTON_ROTATE_SPEED   # = π s for 180°

DOOR_DWELL_S         = 0.5     # how long we hold the button down
DOOR_WAIT_S          = 3.0     # how long we wait after release before probing
DOOR_PROBE_POSE      = (2.11, 7.70, 3.14)   # a point on the OTHER side of the door
MAX_BUTTON_RETRIES   = 3


class ButtonMixin:
    """Open-loop button-press + door-open probe. Stateless — uses
    _open_loop_drive, _open_loop_rotate, _is_reachable from MissionBase."""

    def push_button_and_wait_for_door(self) -> bool:
        """BUTTON_APPROACH assumed reached. Sequence per attempt:
          1. Rotate (180° on first attempt; ~90° re-align on retries)
          2. Back up — depress the button with the rear bumper
          3. Dwell while holding the button
          4. Release: drive forward off the button
          5. Rotate back toward the door
          6. Wait for door to open, then probe DOOR_PROBE_POSE for reachability
             - reachable → door is open, return True
             - not reachable → retry, up to MAX_BUTTON_RETRIES

        Returns True if the door eventually opened."""
        for attempt in range(1, MAX_BUTTON_RETRIES + 1):
            backtrack_time = BUTTON_BACKOFF_TIME + (attempt - 1) * BUTTON_BACKOFF_STEP

            if attempt == 1:
                self.get_logger().info(
                    f'BUTTON: attempt {attempt}/{MAX_BUTTON_RETRIES} — rotate 180°')
                self._open_loop_rotate(BUTTON_ROTATE_SPEED, BUTTON_ROTATE_TIME)
            else:
                self.get_logger().info(
                    f'BUTTON: attempt {attempt}/{MAX_BUTTON_RETRIES} — rotate ~90° to re-align')
                self._open_loop_rotate(BUTTON_ROTATE_SPEED / 2, BUTTON_ROTATE_TIME)

            self.get_logger().info('BUTTON: backing into button')
            self._open_loop_drive(-BUTTON_PUSH_SPEED, backtrack_time)

            self.get_logger().info(f'BUTTON: dwelling on button for {DOOR_DWELL_S}s')
            time.sleep(DOOR_DWELL_S)

            self.get_logger().info('BUTTON: releasing (drive forward off button)')
            self._open_loop_drive(BUTTON_PUSH_SPEED, BUTTON_BACKOFF_TIME)

            self.get_logger().info('BUTTON: rotate back toward door')
            self._open_loop_rotate(-BUTTON_ROTATE_SPEED / 2, BUTTON_ROTATE_TIME)

            self.get_logger().info(f'BUTTON: waiting {DOOR_WAIT_S:.1f}s for door to open')
            time.sleep(DOOR_WAIT_S)

            self.get_logger().info(f'BUTTON: probing reachability to {DOOR_PROBE_POSE}')
            if self._is_reachable(DOOR_PROBE_POSE):
                self.get_logger().info('BUTTON: door open (probe succeeded)')
                return True
            self.get_logger().warn('BUTTON: probe failed — door still closed, retrying')

        self.get_logger().error(
            f'BUTTON: door never opened after {MAX_BUTTON_RETRIES} attempts')
        return False
