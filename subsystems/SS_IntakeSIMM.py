import wpilib
import commands2
import rev
from commands2.button import CommandXboxController

class SS_IntakeSIMM(commands2.Subsystem):
    def __init__(self, motor_id: int, joystick: CommandXboxController):
        super().__init__()
        self.motor = rev.SparkMax(deviceID=motor_id, type=rev.SparkLowLevel.MotorType.kBrushed)
        self._config = rev.SparkMaxConfig()
        self._config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self._config.smartCurrentLimit(30) # amps

        self.motor.configure(self._config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        # self.motor.setInverted(False)
        self.encoder = self.motor.getEncoder()
        self.controller = self.motor.getClosedLoopController()

        self.cruising_speed_factor = .62
        self.max_rpm = 5000

        self._joystick = joystick
        self._joystick.a().whileTrue(self.cruising_speed_command())

    def periodic(self): # Special function called periodically by the robot
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Intake Actual Velocity", self.encoder.getVelocity())
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Intake Setpoint Velocity", self.controller.getSetpoint())

    # -------------------------
    # Motor movement functions
    # -------------------------
    def set_speed(self, speed: float) -> None:
        self.motor.set(speed)

    def stop_motor(self):
        self.motor.set(0)

    # -------------------------
    # Commands
    # -------------------------
    def cruising_speed_command(self):
        return commands2.cmd.startEnd(lambda: self.set_speed(self.cruising_speed_factor), 
                                      lambda: self.stop_motor(), self)
    
    def full_speed_command(self):
        return commands2.cmd.startEnd(lambda: self.set_speed(1), 
                                      lambda: self.stop_motor(), self)
    
    def stop_motor_command(self):
        return commands2.cmd.runOnce(lambda: self.stop_motor(), self)
    
