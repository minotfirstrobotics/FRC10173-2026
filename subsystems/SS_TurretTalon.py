import wpilib
import commands2
import phoenix6
from commands2.button import CommandXboxController

class SS_TurretTalon(commands2.Subsystem):
    def __init__(self, joystick: CommandXboxController):
        super().__init__()
        self.motor = phoenix6.hardware.TalonFX(device_id=1)
        self.requested_power = phoenix6.controls.DutyCycleOut(0)

        cfg = phoenix6.configs.TalonFXConfiguration()
        cfg.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.COAST #BRAKE
        cfg.motor_output.inverted = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        # ENCODER SOFT LIMITS
        # Convert degrees to rotations: 1 rotation = 360°
        self.min_rot = -360.0 / 360.0
        self.max_rot =  3600.0 / 360.0

        cfg.software_limit_switch.forward_soft_limit_enable = True
        cfg.software_limit_switch.reverse_soft_limit_enable = True
        cfg.software_limit_switch.forward_soft_limit_threshold = self.max_rot
        cfg.software_limit_switch.reverse_soft_limit_threshold = self.min_rot

        status = self.motor.configurator.apply(cfg)
        if not status.is_ok():
            wpilib.reportError(f"TalonFX configuration failed: {status}", False)

        # VERY slow turret speed
        self.speed_cap = 0.05

        self._joystick = joystick
        self._joystick.rightBumper().whileTrue(self.run_right_command())
        self._joystick.leftBumper().whileTrue(self.run_left_command())
        # self._joystick.x().onTrue(commands2.cmd.runOnce(self.stop_motor_command))
        # self.joystick.povUp().onTrue(self.ss_turret_talon.go_to_destination_B_command())
        # self.joystick.povDown().onTrue(self.ss_turret_talon.go_to_destination_A_command())
        # self.joystick.povLeft().onTrue(self.ss_turret_talon.stop_motor_command())

    def periodic(self):
        self.position = self.motor.get_rotor_position().value
        wpilib.SmartDashboard.putNumber("Turret Position", self.position)
        # self.is_running = self.velocity > 1e-4
        # wpilib.SmartDashboard.putBoolean("Turret Running", self.is_running)

    # -------------------------
    # Motor movement functions
    # -------------------------
    def set_speed(self, speed: float) -> None:
        clamped = max(-1.0, min(1.0, float(speed)))
        self.motor.set_control(self.requested_power.with_output(clamped))

    def stop_motor(self):
        self.is_running = False
        self.set_speed(0)

    # -------------------------
    # Commands
    # -------------------------
    def run_left_command(self):
        return commands2.cmd.startEnd(lambda: self.set_speed(-self.speed_cap), self.stop_motor, self)

    def run_right_command(self):
        return commands2.cmd.startEnd(lambda: self.set_speed(self.speed_cap), self.stop_motor, self)

    def stop_motor_command(self):
        return commands2.cmd.runOnce(self.stop_motor, self)


