import wpilib
import commands2
import rev
from pathplannerlib.auto import NamedCommands
from commands2.button import CommandXboxController

class SS_ShooterNEO(commands2.Subsystem):
    def __init__(self, motor_id: int, joystick: CommandXboxController):
        super().__init__()
        self.motor = rev.SparkMax(deviceID=motor_id, type=rev.SparkLowLevel.MotorType.kBrushless)
        self._config = rev.SparkMaxConfig()
        self._config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        self._config.smartCurrentLimit(30) # amps
        # self.motor.setInverted(False)
        self.encoder = self.motor.getEncoder()
        self.controller = self.motor.getClosedLoopController()

        self.P = 0.002
        self.I = 0.0000005
        self.D = 0.005
        self.FF = 0.0
        self._config.closedLoop.pidf(self.P, self.I, self.D, self.FF, rev.ClosedLoopSlot.kSlot0)
        self.motor.configure(self._config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        wpilib.SmartDashboard.putNumber("PIDF/Shooter P", self.P)
        wpilib.SmartDashboard.putNumber("PIDF/Shooter I", self.I)
        wpilib.SmartDashboard.putNumber("PIDF/Shooter D", self.D)
        wpilib.SmartDashboard.putNumber("PIDF/Shooter FF", self.FF)

        self.current_velocity = 0.0
        self.setpoint_velocity = 4200 # 5676 is empirical max RPM for NEO
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Shooter Current Velocity ", self.current_velocity)
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Shooter Setpoint Velocity", self.setpoint_velocity)

        self._joystick = joystick
        self._joystick.y().whileTrue(self.run_setpoint_velocity_command())
        # self._joystick.x().onFalse(self.spin_up_and_wait_command())

        NamedCommands.registerCommand("Shooter Spin-up to Setpoint", self.spin_up_and_wait_command())
        NamedCommands.registerCommand("Shooter Stop", self.stop_motor_command())

    def periodic(self): # Special function called periodically by the robot
        if wpilib.SmartDashboard.getNumber("SS_Telemetry/Shooter Setpoint Velocity", self.setpoint_velocity) != self.setpoint_velocity:
            self.setpoint_velocity = wpilib.SmartDashboard.getNumber("SS_Telemetry/Shooter Setpoint Velocity", self.setpoint_velocity)
            # place to update any commands that rely on setpoint_velocity if needed
        if self.setpoint_velocity > 5676: # Cap at empirical max RPM for NEO
            self.setpoint_velocity = 5676
            wpilib.SmartDashboard.putNumber("SS_Telemetry/Shooter Setpoint Velocity", self.setpoint_velocity)
        self.current_velocity = self.encoder.getVelocity()
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Shooter Current Velocity ", self.current_velocity)
        if (wpilib.SmartDashboard.getNumber("PIDF/Shooter P", 0.1) != self.P
        or wpilib.SmartDashboard.getNumber("PIDF/Shooter I", 0.0) != self.I
        or wpilib.SmartDashboard.getNumber("PIDF/Shooter D", 0.0) != self.D
        or wpilib.SmartDashboard.getNumber("PIDF/Shooter FF", 0.0) != self.FF):
            self.P = wpilib.SmartDashboard.getNumber("PIDF/Shooter P", 0.1)
            self.I = wpilib.SmartDashboard.getNumber("PIDF/Shooter I", 0.0)
            self.D = wpilib.SmartDashboard.getNumber("PIDF/Shooter D", 0.0)
            self.FF = wpilib.SmartDashboard.getNumber("PIDF/Shooter FF", 0.0)
            self._config.closedLoop.pidf(self.P, self.I, self.D, self.FF, rev.ClosedLoopSlot.kSlot0)
            self.motor.configure(self._config, rev.ResetMode.kNoResetSafeParameters, rev.PersistMode.kNoPersistParameters)

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
    def run_setpoint_velocity_command(self):
        return commands2.cmd.startEnd(lambda: self.set_velocity(self.setpoint_velocity), 
                                      lambda: self.stop_motor(), self)

    def stop_motor_command(self):
        return commands2.cmd.runOnce(lambda: self.stop_motor(), self)

    def spin_up_and_wait_command(self):
        return SpinUpAndWait_CommDef(self)
    

class SpinUpAndWait_CommDef(commands2.Command):
    def __init__(self, ss_shooter: SS_ShooterNEO):
        super().__init__()
        self.ss_shooter = ss_shooter
        self.addRequirements(ss_shooter) # Ensure no other command uses ss_shooter
        self.timer = wpilib.Timer()
        self.velocity_tolerance = 100 # RPM tolerance for considering the shooter "up to speed"

    def initialize(self):
        self.timer.restart()
        self.ss_shooter.run_setpoint_velocity_command() # Spin up

    def execute(self):
        pass # Command is running

    def isFinished(self):
        return abs(self.ss_shooter.current_velocity-self.ss_shooter.setpoint_velocity) < self.velocity_tolerance

    def end(self, interrupted):
        # Keep spinning even if interrupted, since this command is just for waiting until up to speed.
        if interrupted:
            wpilib.reportWarning("SpinUpAndWait_Command was interrupted before reaching target velocity!", stacktrace=False)
        else:
            wpilib.reportWarning(f"SpinUpAndWait_Command reached target velocity: {self.timer.get():.1f} seconds.", stacktrace=False)
        
