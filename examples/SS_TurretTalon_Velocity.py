import wpilib
import commands2
import phoenix6
from commands2.button import CommandXboxController

class SS_TurretTalon(commands2.Subsystem):
    def __init__(self, motor_id: int, joystick: CommandXboxController):
        super().__init__()
        self.motor = phoenix6.hardware.TalonFX(device_id=motor_id)
        self.requested_power = phoenix6.controls.DutyCycleOut(0)

        cfg = phoenix6.configs.TalonFXConfiguration()
        cfg.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.COAST #BRAKE
        cfg.motor_output.inverted = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        # ENCODER SOFT LIMITS for speed control. Convert degrees to rotations: 1 rotation = 360°
        cfg.software_limit_switch.forward_soft_limit_enable = True
        cfg.software_limit_switch.reverse_soft_limit_enable = True
        cfg.software_limit_switch.forward_soft_limit_threshold = 360.0 / 360.0
        cfg.software_limit_switch.reverse_soft_limit_threshold = -360.0 / 360.0

        status = self.motor.configurator.apply(cfg)
        if not status.is_ok():
            wpilib.reportError(f"TalonFX configuration failed: {status}", False)

        self.motor.set_position(0) # pidf reset encoder position to 0 on startup
        self.cruising_speed = 0.05 # VERY slow turret speed for manual control

        self._joystick = joystick
        self._joystick.rightBumper().whileTrue(self.turn_right_command())
        self._joystick.leftBumper().whileTrue(self.turn_left_command())

    def periodic(self):
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Turret Actual Position", self.motor.get_rotor_position().value)
        # No Position setpoint using the velocity method

    # -------------------------
    # Motor movement functions
    # -------------------------
    def set_speed(self, speed: float) -> None:
        clamped = max(-1.0, min(1.0, float(speed)))
        self.motor.set_control(self.requested_power.with_output(clamped))

    def stop_motor(self):
        self.motor.set(0)

    # -------------------------
    # Commands
    # -------------------------
    def turn_left_command(self):
        return commands2.cmd.startEnd(lambda: self.set_speed(-self.cruising_speed), 
                                      lambda: self.stop_motor(), self)

    def turn_right_command(self):
        return commands2.cmd.startEnd(lambda: self.set_speed(self.cruising_speed), 
                                      lambda: self.stop_motor(), self)
    
    def stop_motor_command(self):
        return commands2.cmd.runOnce(lambda: self.stop_motor(), self)


