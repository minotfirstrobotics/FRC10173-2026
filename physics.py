from __future__ import annotations

from typing import Any

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain
from wpimath.units import metersToFeet

from generated.tuner_constants_2026_GF import TunerConstants


class PhysicsEngine:
    """pyfrc physics engine that mirrors CTRE swerve module states in sim."""

    def __init__(self, physics_controller: PhysicsInterface, robot: Any):
        self.physics_controller = physics_controller
        self.robot = robot

        self._drivetrain = None

        # pyfrc expects wheelbase dimensions in feet.
        self._x_wheelbase_ft = metersToFeet(
            abs(TunerConstants._front_left_y_pos - TunerConstants._front_right_y_pos)
        )
        self._y_wheelbase_ft = metersToFeet(
            abs(TunerConstants._front_left_x_pos - TunerConstants._back_left_x_pos)
        )
        self._max_module_speed_mps = TunerConstants.speed_at_12_volts

    def _try_get_drivetrain(self):
        if self._drivetrain is not None:
            return self._drivetrain

        container = getattr(self.robot, "container", None)
        if container is None:
            return None

        swerve_subsystem = getattr(container, "ss_swerve_drive", None)
        if swerve_subsystem is None:
            return None

        self._drivetrain = getattr(swerve_subsystem, "drivetrain", None)
        return self._drivetrain

    def update_sim(self, now: float, tm_diff: float) -> None:
        drivetrain = self._try_get_drivetrain()
        if drivetrain is None:
            return

        state = drivetrain.get_state()
        if state is None:
            return

        module_states = getattr(state, "module_states", None)
        if not module_states or len(module_states) < 4:
            return

        # Tuner constants create modules in this order: LF, RF, LB, RB.
        lf, rf, lr, rr = module_states[0], module_states[1], module_states[2], module_states[3]

        if self._max_module_speed_mps == 0:
            return

        lr_motor = lr.speed / self._max_module_speed_mps
        rr_motor = rr.speed / self._max_module_speed_mps
        lf_motor = lf.speed / self._max_module_speed_mps
        rf_motor = rf.speed / self._max_module_speed_mps

        speeds = four_motor_swerve_drivetrain(
            lr_motor=lr_motor,
            rr_motor=rr_motor,
            lf_motor=lf_motor,
            rf_motor=rf_motor,
            lr_angle=lr.angle.degrees(),
            rr_angle=rr.angle.degrees(),
            lf_angle=lf.angle.degrees(),
            rf_angle=rf.angle.degrees(),
            x_wheelbase=int(self._x_wheelbase_ft),
            y_wheelbase=int(self._y_wheelbase_ft),
            speed=int(metersToFeet(self._max_module_speed_mps)),
        )

        self.physics_controller.drive(speeds, tm_diff)