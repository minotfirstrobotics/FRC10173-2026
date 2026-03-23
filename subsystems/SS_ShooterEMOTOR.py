import wpilib
import commands2
import constants
import rev

class SS_ShooterEMOTOR(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.motor = rev.SparkMax(constants.CAN_CHANNELS["SHOOTER_EMOTOR"], rev.SparkLowLevel.MotorType.kBrushless)
        self.controller = self.motor.getClosedLoopController()
        self.encoder = self.motor.getEncoder()

        # Attempt PIDF control (tuning required)
        self.controller.setP(0.1)
        self.controller.setI(0.0)
        self.controller.setD(0.0)
        self.controller.setFF(0.0)

        self.position = 0
        self.encoder.setPosition(self.position)
        self.is_running = False

        self.speed_cap = 1
        self.destination_A = 10
        self.destination_B = 40


    def periodic(self): # Special function called periodically by the robot
        self.position = self.encoder.getPosition()
        wpilib.SmartDashboard.putNumber(constants.DASHBOARD_TITLES["SHOOTER_EMOTOR_POSITION"], self.position)
        wpilib.SmartDashboard.putBoolean(constants.DASHBOARD_TITLES["SHOOTER_EMOTOR_RUNNING"], self.is_running)

    def set_speed(self, speed: float) -> None:
        clamped = max(-1.0, min(1.0, float(speed)))
        self.motor.set(clamped)
        self.is_running = abs(clamped) > 1e-6

    def run_forward(self):
        self.is_running = True
        self.set_speed(self.speed_cap)

    def stop_motor(self):
        self.is_running = False
        self.set_speed(0)

    def go_to_destination(self, destination):
        self.controller.setReference(destination, 
                                     rev.SparkBase.ControlType.kPosition, 
                                     rev.ClosedLoopSlot.kSlot0, 
                                     0, 
                                     rev.SparkClosedLoopController.ArbFFUnits.kVoltage)


    ## Commands
    def run_forward_command(self):
        return commands2.cmd.startEnd(self.run_forward, self.stop_motor, self)
    
    def stop_motor_command(self):
        return commands2.cmd.runOnce(self.stop_motor, self)
    

## Usage:
    # from subsystems.SS_EncodedMotor import SS_EncodedMotor
    # self.ss_encoded_motor = SS_EncodedMotor()
    # self.joystick.povUp().whileTrue(self.ss_encoded_motor.run_forward_command())
    # self.joystick.povUp().onTrue(self.ss_encoded_motor.go_to_destination_B_command())
    # self.joystick.povDown().onTrue(self.ss_encoded_motor.go_to_destination_A_command())
    # self.joystick.povLeft().onTrue(self.ss_encoded_motor.stop_motor_command())
