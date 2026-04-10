from __future__ import annotations

import math
from typing import Any, TYPE_CHECKING, cast

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain
from wpimath.units import metersToFeet

from generated.tuner_constants_2026_GF import TunerConstants

if TYPE_CHECKING:
    from phoenix6.hardware import TalonFX


class _KrakenSim:
    """Simple first-order TalonFX mechanism model for pyfrc simulation."""

    def __init__(self, subsystem_attr: str, default_max_rps: float, time_constant_s: float):
        self._subsystem_attr = subsystem_attr
        self._default_max_rps = default_max_rps
        self._time_constant_s = time_constant_s

        self._subsystem: Any | None = None
        self._motor: TalonFX | None = None
        self._position_rot = 0.0
        self._velocity_rps = 0.0

    def _bind(self, robot: Any) -> bool:
        container = getattr(robot, "container", None)
        if container is None:
            return False

        subsystem = getattr(container, self._subsystem_attr, None)
        if subsystem is None:
            return False

        motor_obj = getattr(subsystem, "motor", None)
        if motor_obj is None:
            return False
        motor = cast("TalonFX", motor_obj)

        if motor is not self._motor:
            self._subsystem = subsystem
            self._motor = motor
            self._position_rot = 0.0
            self._velocity_rps = 0.0

        return True

    def _get_max_rps(self) -> float:
        if self._subsystem is None:
            return self._default_max_rps

        value = getattr(self._subsystem, "max_rps", self._default_max_rps)
        try:
            max_rps = abs(float(value))
        except (TypeError, ValueError):
            max_rps = self._default_max_rps

        if not math.isfinite(max_rps) or max_rps <= 0:
            return self._default_max_rps
        return max_rps

    def _get_commanded_fraction(self) -> float:
        if self._subsystem is not None:
            try:
                command_mode = getattr(self._subsystem, "command_mode", "stopped")
                if command_mode == "percent":
                    percent = float(getattr(self._subsystem, "commanded_power_percent", 0.0))
                    return max(min(percent, 1.0), -1.0)
                if command_mode == "velocity":
                    max_rps = self._get_max_rps()
                    velocity = float(getattr(self._subsystem, "commanded_velocity_setpoint", 0.0))
                    if max_rps > 0:
                        return max(min(velocity / max_rps, 1.0), -1.0)
                    return 0.0
            except Exception:
                pass

        if self._motor is None:
            return 0.0

        # Fallback to the motor's reported output if subsystem command state is unavailable.
        try:
            duty = float(self._motor.get())
            return max(min(duty, 1.0), -1.0)
        except Exception:
            pass

        try:
            volts = float(self._motor.get_motor_voltage().value)
            return max(min(volts / 12.0, 1.0), -1.0)
        except Exception:
            return 0.0

    def update(self, robot: Any, tm_diff: float) -> None:
        if tm_diff <= 0:
            return
        if not self._bind(robot):
            return

        max_rps = self._get_max_rps()
        commanded_fraction = self._get_commanded_fraction()
        target_rps = commanded_fraction * max_rps

        alpha = min(tm_diff / max(self._time_constant_s, 1e-3), 1.0)
        self._velocity_rps += (target_rps - self._velocity_rps) * alpha
        self._position_rot += self._velocity_rps * tm_diff

        motor = self._motor
        if motor is None:
            return

        try:
            sim_state = motor.sim_state
            sim_state.set_rotor_velocity(self._velocity_rps)
            sim_state.set_raw_rotor_position(self._position_rot)
        except Exception:
            # If simulation state is unavailable, skip updates silently.
            return


class _PositionKrakenSim(_KrakenSim):
    """Simple profiled position mechanism model for a TalonFX-controlled arm/servo."""

    def __init__(self, subsystem_attr: str, default_vmax_rps: float, default_amax_rps2: float):
        super().__init__(subsystem_attr, default_vmax_rps, time_constant_s=0.0)
        self._default_amax_rps2 = default_amax_rps2

    def _get_requested_position(self) -> float:
        if self._subsystem is None:
            return 0.0

        value = getattr(self._subsystem, "requested_position", 0.0)
        try:
            requested_position = float(value)
        except (TypeError, ValueError):
            requested_position = 0.0

        if not math.isfinite(requested_position):
            return 0.0
        return requested_position

    def _get_amax_rps2(self) -> float:
        if self._subsystem is None:
            return self._default_amax_rps2

        value = getattr(self._subsystem, "Amax", self._default_amax_rps2)
        try:
            amax_rps2 = abs(float(value))
        except (TypeError, ValueError):
            amax_rps2 = self._default_amax_rps2

        if not math.isfinite(amax_rps2) or amax_rps2 <= 0:
            return self._default_amax_rps2
        return amax_rps2

    def update(self, robot: Any, tm_diff: float) -> None:
        if tm_diff <= 0:
            return
        if not self._bind(robot):
            return

        vmax_rps = self._get_max_rps()
        amax_rps2 = self._get_amax_rps2()
        target_position_rot = self._get_requested_position()

        position_error = target_position_rot - self._position_rot

        if abs(position_error) < 1e-3 and abs(self._velocity_rps) < 1e-3:
            self._position_rot = target_position_rot
            self._velocity_rps = 0.0
        else:
            # Speed profile with braking limit so the mechanism decelerates into target.
            max_velocity_for_stopping = math.sqrt(max(0.0, 2.0 * amax_rps2 * abs(position_error)))
            desired_velocity = min(vmax_rps, max_velocity_for_stopping)
            desired_velocity = math.copysign(desired_velocity, position_error) if position_error != 0 else 0.0

            velocity_delta_limit = amax_rps2 * tm_diff
            velocity_error = desired_velocity - self._velocity_rps
            velocity_step = max(min(velocity_error, velocity_delta_limit), -velocity_delta_limit)
            self._velocity_rps += velocity_step
            self._position_rot += self._velocity_rps * tm_diff

            crossed_target = (
                (position_error > 0 and self._position_rot > target_position_rot)
                or (position_error < 0 and self._position_rot < target_position_rot)
            )
            if crossed_target:
                self._position_rot = target_position_rot
                self._velocity_rps = 0.0

        motor = self._motor
        if motor is None:
            return

        try:
            sim_state = motor.sim_state
            sim_state.set_rotor_velocity(self._velocity_rps)
            sim_state.set_raw_rotor_position(self._position_rot)
        except Exception:
            return


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

        # Mechanism estimates:
        # - Shooter: higher-inertia flywheel (slower spin-up response)
        # - Feeder/Intake: lower-inertia rollers (faster response)
        self._shooter_sim = _KrakenSim("ss_shooter", default_max_rps=100.0, time_constant_s=0.45)
        self._feeder_sim = _KrakenSim("ss_feeder", default_max_rps=70.0, time_constant_s=0.18)
        self._intake_sim = _KrakenSim("ss_intake", default_max_rps=60.0, time_constant_s=0.15)
        self._extend_sim = _PositionKrakenSim("ss_extend", default_vmax_rps=2.0, default_amax_rps2=2.0)

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
        if drivetrain is not None:
            state = drivetrain.get_state()
            module_states = getattr(state, "module_states", None) if state is not None else None

            if module_states and len(module_states) >= 4 and self._max_module_speed_mps != 0:
                # Tuner constants create modules in this order: LF, RF, LB, RB.
                lf, rf, lr, rr = module_states[0], module_states[1], module_states[2], module_states[3]

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

        # Update non-drivetrain TalonFX mechanisms.
        self._shooter_sim.update(self.robot, tm_diff)
        self._feeder_sim.update(self.robot, tm_diff)
        self._intake_sim.update(self.robot, tm_diff)
        self._extend_sim.update(self.robot, tm_diff)
