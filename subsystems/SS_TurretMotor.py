import wpilib
import commands2
import phoenix6
from commands2.button import CommandXboxController

class SS_TurretMotor(commands2.Subsystem):
    def __init__(self, joystick: CommandXboxController):
        super().__init__()
        self.motor = phoenix6.hardware.TalonFX(device_id=1)
        self.requested_power = phoenix6.controls.DutyCycleOut(0)

        # configure motor
        cfg = phoenix6.configs.TalonFXConfiguration()
        cfg.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE
        cfg.motor_output.inverted = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        # ENCODER SOFT LIMITS
        # Example: turret allowed from -50° to +50°
        # Convert degrees to rotations: 1 rotation = 360°
        self.min_rot = -50.0 / 360.0
        self.max_rot =  50.0 / 360.0

        cfg.hardware_limit.forward_soft_limit_enable = True
        cfg.hardware_limit.reverse_soft_limit_enable = True
        cfg.hardware_limit.forward_soft_limit = self.max_rot
        cfg.hardware_limit.reverse_soft_limit = self.min_rot

        status = self.motor.configurator.apply(cfg)
        if not status.is_ok():
            wpilib.reportError(f"TalonFX configuration failed: {status}", False)

        self.position = 0.0
        self.is_running = False

        # VERY slow turret speed
        self.speed_cap = 0.15  # 15% power

        # Controller bindings
        self._joystick = joystick

        # Right bumper → rotate right
        self._joystick.rightBumper().whileTrue(self.run_right_command())

        # Left bumper → rotate left
        self._joystick.leftBumper().whileTrue(self.run_left_command())

        # X button → stop
        self._joystick.x().onTrue(commands2.cmd.runOnce(self.stop_motor_command))

    def periodic(self):
        self.position = self.motor.get_rotor_position().value
        wpilib.SmartDashboard.putNumber("Turret Position", self.position)
        wpilib.SmartDashboard.putBoolean("Turret Running", self.is_running)

    def set_speed(self, speed: float) -> None:
        clamped = max(-1.0, min(1.0, float(speed)))
        self.motor.set_control(self.requested_power.with_output(clamped))

    # -------------------------
    # Motor movement functions
    # -------------------------
    def run_left(self):
        self.is_running = True
        self.set_speed(-self.speed_cap)

    def run_right(self):
        self.is_running = True
        self.set_speed(self.speed_cap)

    def stop_motor(self):
        self.is_running = False
        self.set_speed(0)

    # -------------------------
    # Commands
    # -------------------------
    def run_left_command(self):
        return commands2.cmd.startEnd(self.run_left, self.stop_motor, self)

    def run_right_command(self):
        return commands2.cmd.startEnd(self.run_right, self.stop_motor, self)

    def stop_motor_command(self):
        return commands2.cmd.runOnce(self.stop_motor, self)

