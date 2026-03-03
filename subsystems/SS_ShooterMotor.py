from calendar import c

import wpilib
import commands2
import constants
import rev
from commands2.button import CommandXboxController

class SS_ShooterMotor(commands2.Subsystem):
    def __init__(self, joystick: CommandXboxController):
        super().__init__()
        self.motor = rev.SparkMax(constants.CAN_CHANNELS["SHOOTER_MOTOR"], rev.SparkLowLevel.MotorType.kBrushless)
        config = rev.SparkMaxConfig()
        self.motor.configure(config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.motor.setInverted(False)
        config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        config.smartCurrentLimit(40) # amps
        self.encoder = self.motor.getEncoder()
        self.controller = self.motor.getClosedLoopController()

        # Budget PIDF control (enable if velocity control)
        self.controller.setP(0.1)
        self.controller.setI(0.0)
        self.controller.setD(0.0)
        self.controller.setFF(0.0)
        self.velocity = 0
        self.is_running = False
        self.speed_cap = 1
        self.max_rpm = 5000 # 5000 is example max RPM

        # Shooter motor control
        self._joystick = joystick
        self._joystick.rightTrigger(0.2).whileTrue(self.run_velocity_command(self.speed_cap * self.max_rpm))
        self._joystick.y().onTrue(commands2.cmd.runOnce(self.run_velocity_command, 0 * self.max_rpm))

    def periodic(self): # Special function called periodically by the robot
        self.velocity = self.encoder.getVelocity()
        wpilib.SmartDashboard.putNumber(constants.DASHBOARD_TITLES["SHOOTER_MOTOR_VELOCITY"], self.velocity)
        wpilib.SmartDashboard.putBoolean(constants.DASHBOARD_TITLES["SHOOTER_MOTOR_RUNNING"], self.is_running)

    # def set_speed(self, speed: float) -> None:
    #     clamped = max(-1.0, min(1.0, float(speed)))
    #     self.motor.set(clamped)
    #     self.is_running = abs(clamped) > 1e-6

    # Velocity controls
    def set_velocity(self, rpm: float) -> None:
        target = float(rpm)
        self.controller.setSetpoint(target, rev.SparkLowLevel.ControlType.kVelocity)

    # def run_forward(self) -> None:
    #     self.is_running = True
    #     self.set_speed(self.speed_cap)

    # def stop_motor(self) -> None:
    #     self.is_running = False
    #     self.set_speed(0)

    ## Commands
    # def run_forward_command(self):
    #     return commands2.cmd.startEnd(self.run_forward, self.stop_motor, self)
    
    # def stop_motor_command(self):
    #     return commands2.cmd.runOnce(self.stop_motor, self)
    
    # Velocity command
    def run_velocity_command(self, rpm: float) -> commands2.Command:
        return commands2.cmd.startEnd(lambda: self.set_velocity(rpm), lambda: self.set_velocity(0), self)

## Usage:
    # from subsystems.SS_ShooterMotor import SS_ShooterMotor
    # self.ss_shooter_motor = SS_ShooterMotor()
    # self.joystick.povUp().whileTrue(self.ss_shooter_motor.run_forward_command())
    # self.joystick.povUp().onTrue(self.ss_shooter_motor.go_to_destination_B_command())
    # self.joystick.povDown().onTrue(self.ss_shooter_motor.go_to_destination_A_command())
    # self.joystick.povLeft().onTrue(self.ss_shooter_motor.stop_motor_command())
