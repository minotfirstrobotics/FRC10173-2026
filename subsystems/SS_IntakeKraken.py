import wpilib
import commands2
import phoenix6
from pathplannerlib.auto import NamedCommands

class SS_IntakeKraken(commands2.Subsystem):
    def __init__(self, motor_id: int):
        super().__init__()
        self.motor = phoenix6.hardware.TalonFX(device_id=motor_id)
        self._config = phoenix6.configs.TalonFXConfiguration()
        self._config.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.COAST
        self._config.motor_output.inverted = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE

        status = self.motor.configurator.apply(self._config)
        if not status.is_ok():
            wpilib.reportError(f"Intake config failed: {status}", False)

        self.cruising_speed_factor = .62
        self.max_rpm = 7000

        NamedCommands.registerCommand("Intake Spin", commands2.cmd.startEnd(lambda: self.set_speed(self.cruising_speed_factor), 
                                                                            lambda: self.stop_motor(), self))
        NamedCommands.registerCommand("Intake Reverse", commands2.cmd.startEnd(lambda: self.set_speed(-self.cruising_speed_factor), 
                                                                               lambda: self.stop_motor(), self))
        NamedCommands.registerCommand("Intake Stop", commands2.cmd.runOnce(lambda: self.stop_motor(), self))

    def periodic(self): # Special function called periodically by the robot
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Intake Cruise Speed", self.cruising_speed_factor)
        # wpilib.SmartDashboard.putNumber("SS_Telemetry/Intake Actual Velocity", self._read_encoder_velocity())
        # wpilib.SmartDashboard.putNumber("SS_Telemetry/Intake Setpoint Velocity", self._read_controller_setpoint())

    # -------------------------
    # Motor movement functions
    # -------------------------
    def set_speed(self, speed: float) -> None:
        self.motor.set(speed)

    def stop_motor(self):
        self.motor.set(0)

    
