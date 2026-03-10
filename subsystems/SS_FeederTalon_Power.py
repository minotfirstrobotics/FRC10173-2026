import wpilib
import commands2
import phoenix6
from pathplannerlib.auto import NamedCommands
from commands2.button import CommandXboxController

class SS_FeederTalon_Power(commands2.Subsystem):
    def __init__(self, motor_id: int, joystick: CommandXboxController):
        super().__init__()
        self.motor = phoenix6.hardware.TalonFX(device_id=motor_id)

        # configure motor
        cfg = phoenix6.configs.TalonFXConfiguration()
        cfg.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE
        cfg.motor_output.inverted = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        status = self.motor.configurator.apply(cfg)
        if not status.is_ok():
            wpilib.reportError(f"TalonFX configuration failed: {status}", False)

        self.cruising_speed = .2
        self.requested_speed = 0
        self.requested_power = phoenix6.controls.DutyCycleOut(self.requested_speed)

        self._joystick = joystick
        self._joystick.b().whileTrue(self.run_forward_command())

        NamedCommands.registerCommand("Feeder Spin", self.run_forward_command())
        NamedCommands.registerCommand("Feeder Stop", self.stop_motor_command())

    def periodic(self):  # Special function called periodically by the robot
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Feeder Motor Requested Speed", self.requested_speed)
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Feeder Motor Actual Speed", self.motor.get_rotor_velocity().value)

    # -------------------------
    # Motor movement functions
    # -------------------------
    def set_speed(self, speed: float) -> None:
        self.requested_speed = speed
        clamped = max(-1.0, min(1.0, float(speed)))
        self.motor.set_control(self.requested_power.with_output(clamped))

    def stop_motor(self):
        self.is_running = False
        self.set_speed(0)

    # -------------------------
    # Commands
    # -------------------------
    def run_forward_command(self):
        return commands2.cmd.startEnd(lambda: self.set_speed(self.cruising_speed), 
                                      lambda: self.stop_motor(), self)

    def stop_motor_command(self):
        return commands2.cmd.runOnce(lambda: self.stop_motor(), self)
