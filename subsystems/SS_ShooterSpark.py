import wpilib
import commands2
import rev
from commands2.button import CommandXboxController

class SS_ShooterSpark(commands2.Subsystem):
    def __init__(self, motor_id: int, joystick: CommandXboxController):
        super().__init__()
        self.motor = rev.SparkMax(deviceID=motor_id, type=rev.SparkLowLevel.MotorType.kBrushless)
        self._config = rev.SparkMaxConfig()
        self._config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self._config.smartCurrentLimit(30) # amps

        self.P = 0.1
        self.I = 0.0
        self.D = 0.0
        self.FF = 0.0
        self._config.closedLoop.pidf(self.P, self.I, self.D, self.FF, rev.ClosedLoopSlot.kSlot0)
        self.motor.configure(self._config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        wpilib.SmartDashboard.putNumber("PIDF/Shooter P", 0.1)
        wpilib.SmartDashboard.putNumber("PIDF/Shooter I", 0.0)
        wpilib.SmartDashboard.putNumber("PIDF/Shooter D", 0.0)
        wpilib.SmartDashboard.putNumber("PIDF/Shooter FF", 0.0)

        # self.motor.setInverted(False)
        self.encoder = self.motor.getEncoder()
        self.controller = self.motor.getClosedLoopController()

        self.cruising_speed_factor = .5
        self.max_rpm = 5000 # 5000 is example max RPM

        self._joystick = joystick
        self._joystick.y().whileTrue(self.cruising_speed_command())
        self._joystick.x().whileTrue(self.full_speed_command())

    def periodic(self): # Special function called periodically by the robot
        wpilib.SmartDashboard.putNumber("Shooter Current Velocity ", self.encoder.getVelocity())
        wpilib.SmartDashboard.putNumber("Shooter Setpoint Velocity", self.controller.getSetpoint())
        def set_pidf_from_dashboard():
            self._config.closedLoop.pidf(self.P, self.I, self.D, self.FF, rev.ClosedLoopSlot.kSlot0)
            self.motor.configure(self._config, rev.ResetMode.kNoResetSafeParameters, rev.PersistMode.kNoPersistParameters)
        if wpilib.SmartDashboard.getNumber("PIDF/Shooter P", 0.1) != self.P:
            self.P = wpilib.SmartDashboard.getNumber("PIDF/Shooter P", 0.1)
            set_pidf_from_dashboard()
        if wpilib.SmartDashboard.getNumber("PIDF/Shooter I", 0.0) != self.I:
            self.I = wpilib.SmartDashboard.getNumber("PIDF/Shooter I", 0.0)
            set_pidf_from_dashboard()
        if wpilib.SmartDashboard.getNumber("PIDF/Shooter D", 0.0) != self.D:
            self.D = wpilib.SmartDashboard.getNumber("PIDF/Shooter D", 0.0)
            set_pidf_from_dashboard()
        if wpilib.SmartDashboard.getNumber("PIDF/Shooter FF", 0.0) != self.FF:
            self.FF = wpilib.SmartDashboard.getNumber("PIDF/Shooter FF", 0.0)
            set_pidf_from_dashboard()        

    # -------------------------
    # Motor movement functions
    # -------------------------
    def set_velocity(self, target_rpm: float) -> None:
        self.controller.setSetpoint(float(target_rpm), 
                                    rev.SparkLowLevel.ControlType.kVelocity, 
                                    rev.ClosedLoopSlot.kSlot0)

    def stop_motor(self):
        self.motor.set(0)

    # -------------------------
    # Commands
    # -------------------------
    def run_velocity_command(self, rpm: float) -> commands2.Command:
        return commands2.cmd.startEnd(lambda: self.set_velocity(rpm), 
                                      lambda: self.stop_motor(), self)

    def cruising_speed_command(self):
        return commands2.cmd.startEnd(lambda: self.set_velocity(self.cruising_speed_factor*self.max_rpm), 
                                      lambda: self.stop_motor(), self)
    
    def full_speed_command(self):
        return commands2.cmd.startEnd(lambda: self.set_velocity(1*self.max_rpm), 
                                      lambda: self.stop_motor(), self)

    def stop_motor_command(self):
        return commands2.cmd.runOnce(lambda: self.stop_motor(), self)


