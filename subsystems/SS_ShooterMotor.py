import wpilib
import commands2
import constants
import rev

class SS_ShooterMotor(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.motor = rev.SparkMax(constants.CAN_CHANNELS["SHOOTER_MOTOR"], rev.SparkLowLevel.MotorType.kBrushless)
        self.motor.restoreFactoryDefaults()
        self.motor.setInverted(False)
        self.motor.setIdleMode(rev.IdleMode.kBrake)
        self.encoder = self.motor.getEncoder()
        self.controller = self.motor.getClosedLoopController()
        
        # # Budget PIDF control (enable if velocity control)
        # self.controller.setP(0.1)
        # self.controller.setI(0.0)
        # self.controller.setD(0.0)
        # self.controller.setFF(0.0)
        # self.motor.setSmartCurrentLimit(40) # amps
        self.position = 0
        self.encoder.setPosition(self.position)
        self.is_running = False
        self.speed_cap = 1

    def periodic(self): # Special function called periodically by the robot
        self.position = self.encoder.getPosition()
        wpilib.SmartDashboard.putNumber(constants.DASHBOARD_TITLES["SHOOTER_MOTOR_POSITION"], self.position)
        wpilib.SmartDashboard.putBoolean(constants.DASHBOARD_TITLES["SHOOTER_MOTOR_RUNNING"], self.is_running)

    def set_speed(self, speed: float) -> None:
        clamped = max(-1.0, min(1.0, float(speed)))
        self.motor.set(clamped)
        self.is_running = abs(clamped) > 1e-6

    # # Velocity controls
    # def se_velocity(self, rpm: float) -> None:
    #     target = float(rpm)
    #     self.controller.setReference(target, rev.ControlType.kVelocity)

    def run_forward(self) -> None:
        self.is_running = True
        self.set_speed(self.speed_cap)

    def stop_motor(self) -> None:
        self.is_running = False
        self.set_speed(0)

    ## Commands
    def run_forward_command(self):
        return commands2.cmd.startEnd(self.run_forward, self.stop_motor, self)
    
    def stop_motor_command(self):
        return commands2.cmd.runOnce(self.stop_motor, self)
    
    # # Velocity command
    # def run_velocity_command(self, rpm: float):
    #     # starts velocity control when scheduled, stops motor when ended
    #     return commands2.cmd.startEnd(lambda: self.se_velocity(rpm), lambda: self.set_speed(0), self)

## Usage:
    # from subsystems.SS_ShooterMotor import SS_ShooterMotor
    # self.ss_shooter_motor = SS_ShooterMotor()
    # self.joystick.povUp().whileTrue(self.ss_shooter_motor.run_forward_command())
    # self.joystick.povUp().onTrue(self.ss_shooter_motor.go_to_destination_B_command())
    # self.joystick.povDown().onTrue(self.ss_shooter_motor.go_to_destination_A_command())
    # self.joystick.povLeft().onTrue(self.ss_shooter_motor.stop_motor_command())
