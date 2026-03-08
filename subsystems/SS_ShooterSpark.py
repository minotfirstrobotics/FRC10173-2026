from calendar import c

import wpilib
import commands2
import rev
from commands2.button import CommandXboxController

class SS_ShooterSpark(commands2.Subsystem):
    def __init__(self, joystick: CommandXboxController):
        super().__init__()
        self.motor = rev.SparkMax(deviceID=3, type=rev.SparkLowLevel.MotorType.kBrushless)
        self._config = rev.SparkMaxConfig()
        self._config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self._config.smartCurrentLimit(30) # amps
        self._config.closedLoop.pidf(0.1, 0.1, 0.0, 0.0, rev.ClosedLoopSlot.kSlot0)
        self.motor.configure(self._config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        # self.motor.setInverted(False)
        self.encoder = self.motor.getEncoder()
        self.controller = self.motor.getClosedLoopController()

        self.crusing_speed_factor = .5
        self.max_rpm = 5000 # 5000 is example max RPM

        self._joystick = joystick
        self._joystick.y().whileTrue(self.cruising_speed_command())
        self._joystick.x().whileTrue(self.full_speed_command())

    def periodic(self): # Special function called periodically by the robot
        wpilib.SmartDashboard.putNumber("Shooter Current Velocity ", self.encoder.getVelocity())
        wpilib.SmartDashboard.putNumber("Shooter Setpoint Velocity", self.controller.getSetpoint())

    # -------------------------
    # Motor movement functions
    # -------------------------
    def set_velocity(self, target_rpm: float) -> None:
        self.controller.setSetpoint(float(target_rpm), 
                                    rev.SparkLowLevel.ControlType.kVelocity, 
                                    rev.ClosedLoopSlot.kSlot0
                                    )

    def stop_motor(self):
        self.motor.set(0)

    # def adjust_FF(self, ff: float) -> None:
    #     self._config.closedLoop.pidf(0.1, 0.0, 0.0, ff, rev.ClosedLoopSlot.kSlot0)
    #     self.motor.configure(self._config, 
    #                          rev.ResetMode.kNoResetSafeParameters, 
    #                          rev.PersistMode.kNoPersistParameters)

    # -------------------------
    # Commands
    # -------------------------
    def run_velocity_command(self, rpm: float) -> commands2.Command:
        return commands2.cmd.startEnd(lambda: self.set_velocity(rpm), 
                                      lambda: self.stop_motor(), self)

    def cruising_speed_command(self):
        return commands2.cmd.startEnd(lambda: self.set_velocity(self.crusing_speed_factor*self.max_rpm), 
                                      lambda: self.stop_motor, self)
    
    def full_speed_command(self):
        return commands2.cmd.startEnd(lambda: self.set_velocity(1*self.max_rpm), 
                                      lambda: self.stop_motor, self)

    def stop_motor_command(self):
        return commands2.cmd.runOnce(lambda: self.stop_motor(), self)


