from calendar import c

import wpilib
import commands2
import constants
import rev
from commands2.button import CommandXboxController

class SS_IntakeSystem(commands2.Subsystem):
    def __init__(self, joystick: CommandXboxController):
        super().__init__()
        self.motor = rev.SparkMax(constants.CAN_CHANNELS["INTAKE_SYSTEM"], rev.SparkLowLevel.MotorType.kBrushed)
        self._config = rev.SparkMaxConfig()
        self._config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self._config.smartCurrentLimit(30) # amps
        self._config.closedLoop.pidf(0.1, 0.0, 0.0, 0.0, rev.ClosedLoopSlot.kSlot0)
        self.motor.configure(self._config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.motor.setInverted(False)
        self.encoder = self.motor.getEncoder()
        self.controller = self.motor.getClosedLoopController()

        self.power = 0
        self.is_running = False
        self.speed_cap = 1

        # Intake controls
        self._joystick = joystick
        self._joystick.rightTrigger(0.2).whileTrue(self.run_forward_command())
        self._joystick.y().onTrue(self.stop_motor_command())

    def periodic(self): # Special function called periodically by the robot
        self.position = self.encoder.getPosition()
        wpilib.SmartDashboard.putNumber(constants.DASHBOARD_TITLES["INTAKE_SYSTEM_POSITION"], self.position)
        wpilib.SmartDashboard.putBoolean(constants.DASHBOARD_TITLES["INTAKE_SYSTEM_RUNNING"], self.is_running)

    def set_speed(self, speed: float) -> None:
        clamped = max(-1.0, min(1.0, float(speed)))
        self.motor.set(clamped)
        self.is_running = abs(clamped) > 1e-6

    # # Velocity controls
    # def set_velocity(self, rpm: float) -> None:
    #     target = float(rpm)
    #     self.controller.setSetpoint(target, 
    #                                 rev.SparkLowLevel.ControlType.kVelocity, 
    #                                 rev.ClosedLoopSlot.kSlot0
    #                                 )

    # def adjust_FF(self, ff: float) -> None:
    #     self._config.closedLoop.pidf(0.1, 0.0, 0.0, ff, rev.ClosedLoopSlot.kSlot0)
    #     self.motor.configure(self._config, 
    #                          rev.ResetMode.kNoResetSafeParameters, 
    #                          rev.PersistMode.kNoPersistParameters)

    def run_forward(self) -> None:
        self.is_running = True
        self.set_speed(self.speed_cap)

    def stop_motor(self) -> None:
        self.is_running = False
        self.set_speed(0)

    # Commands
    def run_forward_command(self):
        return commands2.cmd.startEnd(self.run_forward, self.stop_motor, self)
    
    def stop_motor_command(self):
        return commands2.cmd.runOnce(self.stop_motor, self)
    
    # # Velocity command
    # def run_velocity_command(self, rpm: float) -> commands2.Command:
    #     return commands2.cmd.startEnd(lambda: self.set_velocity(rpm), lambda: self.set_velocity(0), self)

## Usage:
    # from subsystems.SS_IntakeSystem import SS_IntakeSystem
    # self.ss_intake_system = SS_IntakeSystem(self.joystick)
    # self.joystick.povUp().whileTrue(self.ss_intake_system.run_forward_command())
    # self.joystick.povUp().onTrue(self.ss_intake_system.go_to_destination_B_command())
    # self.joystick.povDown().onTrue(self.ss_intake_system.go_to_destination_A_command())
    # self.joystick.povLeft().onTrue(self.ss_intake_system.stop_motor_command())
