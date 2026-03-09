import wpilib
import commands2
import rev
from commands2.button import CommandXboxController

class SS_IntakeSpark(commands2.Subsystem):
    def __init__(self, motor_id: int, joystick: CommandXboxController):
        super().__init__()
        self.motor = rev.SparkMax(deviceID=motor_id, type=rev.SparkLowLevel.MotorType.kBrushed)
        self._config = rev.SparkMaxConfig()
        self._config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self._config.smartCurrentLimit(30) # amps

        """self.P = 0.1
        self.I = 0.0
        self.D = 0.0
        self.FF = 0.0
        wpilib.SmartDashboard.putNumber("PIDF/Intake P", 0.1)
        wpilib.SmartDashboard.putNumber("PIDF/Intake I", 0.0)
        wpilib.SmartDashboard.putNumber("PIDF/Intake D", 0.0)
        wpilib.SmartDashboard.putNumber("PIDF/Intake FF", 0.0)
        self._config.closedLoop.pidf(self.P, self.I, self.D, self.FF, rev.ClosedLoopSlot.kSlot0)"""
        self.motor.configure(self._config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        # self.motor.setInverted(False)
        self.encoder = self.motor.getEncoder()
        self.controller = self.motor.getClosedLoopController()

        self.cruising_speed_factor = .1
        self.max_rpm = 5000

        self._joystick = joystick
        self._joystick.a().whileTrue(self.cruising_speed_command())

    def periodic(self): # Special function called periodically by the robot
        wpilib.SmartDashboard.putNumber("Intake Actual Velocity", self.encoder.getVelocity())
        wpilib.SmartDashboard.putNumber("Intake Setpoint Velocity", self.controller.getSetpoint())
        """def set_pidf_from_dashboard():
            self._config.closedLoop.pidf(self.P, self.I, self.D, self.FF, rev.ClosedLoopSlot.kSlot0)
            self.motor.configure(self._config, rev.ResetMode.kNoResetSafeParameters, rev.PersistMode.kNoPersistParameters)
        if wpilib.SmartDashboard.getNumber("PIDF/Intake P", 0.1) != self.P:
            self.P = wpilib.SmartDashboard.getNumber("PIDF/Intake P", 0.1)
            set_pidf_from_dashboard()
        if wpilib.SmartDashboard.getNumber("PIDF/Intake I", 0.0) != self.I:
            self.I = wpilib.SmartDashboard.getNumber("PIDF/Intake I", 0.0)
            set_pidf_from_dashboard()
        if wpilib.SmartDashboard.getNumber("PIDF/Intake D", 0.0) != self.D:
            self.D = wpilib.SmartDashboard.getNumber("PIDF/Intake D", 0.0)
            set_pidf_from_dashboard()
        if wpilib.SmartDashboard.getNumber("PIDF/Intake FF", 0.0) != self.FF:
            self.FF = wpilib.SmartDashboard.getNumber("PIDF/Intake FF", 0.0)
            set_pidf_from_dashboard()"""

    # -------------------------
    # Motor movement functions
    # -------------------------
    def set_velocity(self, target_factor: float) -> None:
        self.controller.setSetpoint(float(target_factor * self.max_rpm), 
                                    rev.SparkLowLevel.ControlType.kVelocity, 
                                    rev.ClosedLoopSlot.kSlot0)

    def set_speed(self, speed: float) -> None:
        self.motor.set(speed)

    def stop_motor(self):
        self.motor.set(0)

    # -------------------------
    # Commands
    # -------------------------
    def run_velocity_command(self, factor: float) -> commands2.Command:
        return commands2.cmd.startEnd(lambda: self.set_velocity(factor), 
                                      lambda: self.stop_motor(), self)

    def cruising_speed_command(self):
        return commands2.cmd.startEnd(lambda: self.set_velocity(self.cruising_speed_factor), 
                                      lambda: self.stop_motor(), self)
    
    def full_speed_command(self):
        return commands2.cmd.startEnd(lambda: self.set_velocity(1), 
                                      lambda: self.stop_motor(), self)
    
    def stop_motor_command(self):
        return commands2.cmd.runOnce(lambda: self.stop_motor(), self)
    
