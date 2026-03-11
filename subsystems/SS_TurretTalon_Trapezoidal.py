import wpilib
import commands2
import phoenix6
from pathplannerlib.auto import NamedCommands
from commands2.button import CommandXboxController

class SS_TurretTalon(commands2.Subsystem):
    def __init__(self, motor_id: int, joystick: CommandXboxController):
        super().__init__()
        self.motor = phoenix6.hardware.TalonFX(device_id=motor_id)
        self.requested_power = phoenix6.controls.DutyCycleOut(0)

        self.cfg = phoenix6.configs.TalonFXConfiguration()
        self.cfg.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE
        self.cfg.motor_output.inverted = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        # Setup Motion Magic control mode for position control with trapezoidal motion profiling
        self.cfg.slot0.k_p = 5.0
        self.cfg.slot0.k_i = 0.0
        self.cfg.slot0.k_d = 0.0
        self.cfg.slot0.k_v = 0.0  # optional velocity feedforward for running at certain speeds
        self.cfg.slot0.k_s = 0.0  # optional static feedforward for overcoming static friction
        self.cfg.slot0.k_a = 0.0  # optional acceleration feedforward for compensating inertia
        self.cfg.slot0.k_g = 0.0  # optional gravity feedforward for compensating gravity effects
        self.cfg.motion_magic.motion_magic_cruise_velocity = 2.0      # rps
        self.cfg.motion_magic.motion_magic_acceleration = 2.0         # rps^2
        self.cfg.motion_magic.motion_magic_jerk = 10.0                # rps^3 (optional)
        self.cfg.feedback.sensor_to_mechanism_ratio = 100.0
        wpilib.SmartDashboard.putNumber("PIDF/Turret P", 0.1)
        wpilib.SmartDashboard.putNumber("PIDF/Turret I", 0.0)
        wpilib.SmartDashboard.putNumber("PIDF/Turret D", 0.0)
        wpilib.SmartDashboard.putNumber("PIDF/Turret Fv", 0.0)
        wpilib.SmartDashboard.putNumber("PIDF/Turret Fs", 0.0)
        wpilib.SmartDashboard.putNumber("PIDF/Turret Fa", 0.0)
        wpilib.SmartDashboard.putNumber("PIDF/Turret Fg", 0.0)
        wpilib.SmartDashboard.putNumber("PIDF/Turret Vcruise", 2.0)
        wpilib.SmartDashboard.putNumber("PIDF/Turret Accel", 2.0)
        wpilib.SmartDashboard.putNumber("PIDF/Turret Fjerk", 10.0)
        self.position_request_with_trapezoid = phoenix6.controls.MotionMagicDutyCycle(0.0)

        status = self.motor.configurator.apply(self.cfg)
        if not status.is_ok():
            wpilib.reportError(f"TalonFX configuration failed: {status}", False)

        self.requested_position = 0.0
        self.motor.set_position(self.requested_position) # pidf reset encoder position to 0 on startup
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Turret Actual Position", 0.0)
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Turret Setpoint Position", 0.0)

        self._joystick = joystick
        self._joystick.leftBumper().onTrue(self.point_ahead_command())
        self._joystick.rightBumper().onTrue(self.point_right_command())

        NamedCommands.registerCommand("Turret Point Ahead", self.point_ahead_command())
        NamedCommands.registerCommand("Turret Point Right", self.point_right_command())

    def periodic(self):
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Turret Actual Position", self.motor.get_rotor_position().value)
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Turret Setpoint Position", self.requested_position)
        if wpilib.SmartDashboard.getNumber("PIDF/Turret P", 0.1) != self.cfg.slot0.k_p:
            self.cfg.slot0.k_p = wpilib.SmartDashboard.getNumber("PIDF/Turret P", 0.1)
        if wpilib.SmartDashboard.getNumber("PIDF/Turret I", 0.0) != self.cfg.slot0.k_i:
            self.cfg.slot0.k_i = wpilib.SmartDashboard.getNumber("PIDF/Turret I", 0.0)
        if wpilib.SmartDashboard.getNumber("PIDF/Turret D", 0.0) != self.cfg.slot0.k_d:
            self.cfg.slot0.k_d = wpilib.SmartDashboard.getNumber("PIDF/Turret D", 0.0)
        if wpilib.SmartDashboard.getNumber("PIDF/Turret Fv", 0.0) != self.cfg.slot0.k_v:
            self.cfg.slot0.k_v = wpilib.SmartDashboard.getNumber("PIDF/Turret Fv", 0.0)
        if wpilib.SmartDashboard.getNumber("PIDF/Turret Fs", 0.0) != self.cfg.slot0.k_s:
            self.cfg.slot0.k_s = wpilib.SmartDashboard.getNumber("PIDF/Turret Fs", 0.0)
        if wpilib.SmartDashboard.getNumber("PIDF/Turret Fa", 0.0) != self.cfg.slot0.k_a:
            self.cfg.slot0.k_a = wpilib.SmartDashboard.getNumber("PIDF/Turret Fa", 0.0)
        if wpilib.SmartDashboard.getNumber("PIDF/Turret Fg", 0.0) != self.cfg.slot0.k_g:
            self.cfg.slot0.k_g = wpilib.SmartDashboard.getNumber("PIDF/Turret Fg", 0.0)
        if wpilib.SmartDashboard.getNumber("PIDF/Turret Vcruise", 2.0) != self.cfg.motion_magic.motion_magic_cruise_velocity:
            self.cfg.motion_magic.motion_magic_cruise_velocity = wpilib.SmartDashboard.getNumber("PIDF/Turret Vcruise", 2.0)
        if wpilib.SmartDashboard.getNumber("PIDF/Turret Accel", 2.0) != self.cfg.motion_magic.motion_magic_acceleration:
            self.cfg.motion_magic.motion_magic_acceleration = wpilib.SmartDashboard.getNumber("PIDF/Turret Accel", 2.0)
        if wpilib.SmartDashboard.getNumber("PIDF/Turret Fjerk", 10.0) != self.cfg.motion_magic.motion_magic_jerk:
            self.cfg.motion_magic.motion_magic_jerk = wpilib.SmartDashboard.getNumber("PIDF/Turret Fjerk", 10.0)


    # -------------------------
    # Motor movement functions
    # -------------------------
    def set_position(self, target_rotations: float) -> None:
        self.motor.set_control(self.position_request_with_trapezoid.with_position(float(target_rotations)))
        self.requested_position = target_rotations

    def stop_motor(self):
        self.motor.set(0)

    # -------------------------
    # Commands
    # -------------------------
    def point_ahead_command(self):
        return commands2.cmd.runOnce(lambda: self.set_position(0), self)
    
    def point_right_command(self):
        return commands2.cmd.runOnce(lambda: self.set_position(2.4), self)
    
    def point_behind_command(self):
        return commands2.cmd.runOnce(lambda: self.set_position(4.7), self)
    
    def point_to_target_command(self, target_rotations: float):
        return commands2.cmd.runOnce(lambda: self.set_position(target_rotations), self)
    
    def stop_motor_command(self):
        return commands2.cmd.runOnce(lambda: self.stop_motor(), self)


