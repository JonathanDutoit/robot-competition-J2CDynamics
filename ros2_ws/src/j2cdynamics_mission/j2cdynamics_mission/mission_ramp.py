"""
mission_ramp — open-loop ramp climb/descent sequence.

Used by missions that need to traverse the arena ramp (currently
do_mission.py — Duplo-Obliterator). Stateless mixin: no __init__, just
methods + constants. Mix in after DuploMixin, before MissionBase:

    class DoMissionRunner(DuploMixin, RampMixin, MissionBase): ...

The sequence is open-loop because the ramp surface confuses odometry/AMCL
and Nav2 can't reliably plan over it. The mission is responsible for getting
the robot to a known approach pose first (e.g. via _go_to_with_recovery);
the climb itself is just twists on ramp_vel (twist_mux priority 150, so it
preempts both visual servo and Nav2).

Tuning knobs are at the top — set them by trial on the actual arena ramp.
"""

import math


# ── Ramp constants (tune on hardware) ────────────────────────────────────────
RAMP_BACKOFF_TIME    = 1.5            # s — reverse step that depresses the button
RAMP_ROTATE_SPEED    = 1.0            # rad/s for the 180° spin
RAMP_ROTATE_TIME     = math.pi / RAMP_ROTATE_SPEED   # = π s for 180°

RAMP_SPEED           = 0.35           # m/s — climb speed
RAMP_TIME            = 6.5            # s   — climb duration

RAMP_DOWN_SPEED      = 0.10           # m/s — descent (positive; sign set at call site)
RAMP_DOWN_TIME       = 12.0           # s

RAMP_DOWN_FOWARD_SPEED = 0.20
RAMP_DOWN_FORWARD_TIME = 3


class RampMixin:
    """Open-loop ramp climb/descent. Stateless — uses _open_loop_drive and
    _open_loop_rotate from MissionBase."""

    def go_up_ramp(self) -> bool:
        """Approach pose assumed reached. Sequence:
          1. Rotate 180° in place (so back of robot/scoop faces the button)
          2. Reverse — push the ramp button with the rear bumper
          3. Rotate ~120° back to face the ramp face
          4. Drive forward up the ramp (open-loop, ramp_vel)
        Returns True (success not actually validated — open-loop)."""
        self.get_logger().info('Ramp: rotate 180°')
        self._open_loop_rotate(RAMP_ROTATE_SPEED, RAMP_ROTATE_TIME)

        self.get_logger().info('Ramp: backing into ramp wall')
        self._open_loop_drive(-RAMP_SPEED, RAMP_BACKOFF_TIME)

        self.get_logger().info('Ramp: rotate back toward ramp')
        self._open_loop_rotate(-RAMP_ROTATE_SPEED / 1.45, RAMP_ROTATE_TIME)

        self.get_logger().info('Ramp: drive up')
        self._open_loop_drive(RAMP_SPEED, RAMP_TIME)
        return True

    def go_down_ramp(self) -> bool:
        """Open-loop descent. Sign of RAMP_DOWN_SPEED is positive here;
        caller's intent is "drive backward off the ramp" so we pass negative."""
        self.get_logger().info('Ramp: descending')

        self.get_logger().info('Ramp: going back to the wall')
        self._open_loop_drive(-RAMP_SPEED, RAMP_BACKOFF_TIME)

        self.get_logger().info('Ramp: rotate back toward ramp')
        self._open_loop_rotate(RAMP_ROTATE_SPEED / 1.4, RAMP_ROTATE_TIME)

        self.get_logger().info('Ramp: going down the ramp')
        self._open_loop_drive(RAMP_DOWN_SPEED, RAMP_DOWN_TIME * 1.5)
        
        self.get_logger().info('Ramp: going forward')
        self._open_loop_drive(RAMP_DOWN_FOWARD_SPEED, RAMP_DOWN_FORWARD_TIME)

        self.get_logger().info('Ramp: rotate back toward base')
        self._open_loop_rotate(-RAMP_ROTATE_SPEED / 1.8, RAMP_ROTATE_TIME)
        return True
