import wpilib
import commands2
import constants
import phoenix6
from commands2.button import CommandXboxController

class SS_UptakeMotor(commands2.Subsystem):
    def __init__(self, joystick: CommandXboxController):
        super().__init__()
        self.motor = phoenix6.hardware.TalonFX(constants.CAN_CHANNELS["UPTAKE_MOTOR"])
        self.requested_power = phoenix6.controls.DutyCycleOut(0)

        # configure motor
        cfg = phoenix6.configs.TalonFXConfiguration()
        cfg.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE
        cfg.motor_output.inverted = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        # # Budget PID control (enable if velocity control)
        # cfg.slot0.k_p = 0.1
        # cfg.slot0.k_i = 0.0
        # cfg.slot0.k_d = 0.0
        # # cfg.slot0.k_s = 0.0
        # # cfg.slot0.k_v = 0.0
        status = self.motor.configurator.apply(cfg)
        if not status.is_ok():
            wpilib.reportError(f"TalonFX configuration failed: {status}", False)

        self.position = 0.0
        self.is_running = False
        self.speed_cap = 1

        # Uptake motor control
        self._joystick = joystick
        self._joystick.rightBumper().whileTrue(self.run_forward_command())
        self._joystick.x().onTrue(commands2.cmd.runOnce(self.stop_motor_command))

    def periodic(self):  # Special function called periodically by the robot
        self.position = self.motor.get_rotor_position().value
        wpilib.SmartDashboard.putNumber(constants.DASHBOARD_TITLES["UPTAKE_MOTOR_POSITION"], self.position)
        wpilib.SmartDashboard.putBoolean(constants.DASHBOARD_TITLES["UPTAKE_MOTOR_RUNNING"], self.is_running)

    # # Velocity controls
    # def set_velocity(self, rpm: float) -> None:
    #     target = float(rpm)
    #     self.motor.set_control(phoenix6.controls.VelocityDutyCycle(target))

    def set_speed(self, speed: float) -> None:
        clamped = max(-1.0, min(1.0, float(speed)))
        self.motor.set_control(self.requested_power.with_output(clamped))
        # self.is_running = abs(clamped) > 1e-6

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
    # # starts velocity control when scheduled, stops motor when ended
    #     return commands2.cmd.startEnd(lambda: self.set_velocity(rpm), self.stop_motor, self)
