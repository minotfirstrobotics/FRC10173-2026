import wpilib
import commands2
import phoenix6
from commands2.button import CommandXboxController

class SS_TurretTalon(commands2.Subsystem):
    def __init__(self, motor_id: int, joystick: CommandXboxController):
        super().__init__()
        self.motor = phoenix6.hardware.TalonFX(device_id=motor_id)

        cfg = phoenix6.configs.TalonFXConfiguration()
        cfg.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.COAST #BRAKE
        cfg.motor_output.inverted = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        # Setup PID control for position control
        cfg.slot0.k_p = 5.0
        cfg.slot0.k_i = 0.0
        cfg.slot0.k_d = 0.0
        cfg.slot0.k_v = 0.0  # optional feedforward velocity
        cfg.slot0.k_g = 0.0  # optional gravity feedforward for compensating gravity effects
        self.position_request_with_pidf = phoenix6.controls.PositionDutyCycle(0.0)

        status = self.motor.configurator.apply(cfg)
        if not status.is_ok():
            wpilib.reportError(f"TalonFX configuration failed: {status}", False)

        self.requested_position = 0.0
        self.motor.set_position(self.requested_position) # pidf reset encoder position to 0 on startup

        self._joystick = joystick
        self._joystick.rightBumper().onFalse(self.point_right_command())
        self._joystick.leftBumper().onFalse(self.point_ahead_command())

    def periodic(self):
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Turret Actual Position", self.motor.get_rotor_position().value)
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Turret Setpoint Position", self.requested_position)

    # -------------------------
    # Motor movement functions
    # -------------------------
    def set_position(self, target_rotations: float) -> None:
        self.motor.set_control(self.position_request_with_pidf.with_position(float(target_rotations)))
        self.requested_position = target_rotations

    def stop_motor(self):
        self.motor.set(0)

    # -------------------------
    # Commands
    # -------------------------
    def point_ahead_command(self):
        return commands2.cmd.runOnce(lambda: self.set_position(-0.25), self)
    
    def point_right_command(self):
        return commands2.cmd.runOnce(lambda: self.set_position(0.0), self)
    
    def point_behind_command(self):
        return commands2.cmd.runOnce(lambda: self.set_position(0.25), self)
    
    def point_to_target_command(self, target_rotations: float):
        return commands2.cmd.runOnce(lambda: self.set_position(target_rotations), self)
    
    def stop_motor_command(self):
        return commands2.cmd.runOnce(lambda: self.stop_motor(), self)
