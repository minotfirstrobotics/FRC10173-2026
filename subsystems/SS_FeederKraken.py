import wpilib
import commands2
import phoenix6
from pathplannerlib.auto import NamedCommands

class SS_FeederKraken(commands2.Subsystem):
    def __init__(self, motor_id: int):
        super().__init__()
        self.motor = phoenix6.hardware.TalonFX(device_id=motor_id)
        self._config = phoenix6.configs.TalonFXConfiguration()
        self._config.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.COAST
        self._config.motor_output.inverted = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE

        self._config.current_limits.stator_current_limit_enable = True
        self._config.current_limits.stator_current_limit = 80.0
        self._config.current_limits.supply_current_limit_enable = True
        self._config.current_limits.supply_current_limit = 40.0

        self.P = 0.00017
        self.I = 0.0
        self.D = 0.0
        self.FF = 0.00017
        self.S = 0.0
        self._apply_pidf_to_config()

        wpilib.SmartDashboard.putNumber("PIDF/Feeder P", self.P)
        wpilib.SmartDashboard.putNumber("PIDF/Feeder I", self.I)
        wpilib.SmartDashboard.putNumber("PIDF/Feeder D", self.D)
        wpilib.SmartDashboard.putNumber("PIDF/Feeder FF", self.FF)
        wpilib.SmartDashboard.putNumber("PIDF/Feeder S", self.S)
        status = self.motor.configurator.apply(self._config)
        if not status.is_ok():
            wpilib.reportError(f"Feeder config failed: {status}", False)

        self.velocity_request = phoenix6.controls.VelocityDutyCycle(0.0)
        self.voltage_request  = phoenix6.controls.VoltageOut(0.0)

        self.current_velocity = 0.0
        self.setpoint_velocity = 87.5 # max rpm 7000 so max rps 116.67


        wpilib.SmartDashboard.putNumber("SS_Telemetry/Feeder Current Velocity ", self.current_velocity)
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Feeder Setpoint Velocity", self.setpoint_velocity)
        # NamedCommands.registerCommand("Feeder Spin-up to Setpoint", self.spin_up_and_wait_command())
        NamedCommands.registerCommand("Feeder Setpoint Velocity", commands2.cmd.runOnce(self.run_velocity_at_setpoint))
        NamedCommands.registerCommand("Feeder Stop", commands2.cmd.runOnce(self.stop_motor))

    def _apply_pidf_to_config(self):
        self._config.slot0.k_p = self.P
        self._config.slot0.k_i = self.I
        self._config.slot0.k_d = self.D
        self._config.slot0.k_v = self.FF
        self._config.slot0.k_s = self.S

    def periodic(self): # Special function called periodically by the robot
        if wpilib.SmartDashboard.getNumber("SS_Telemetry/Feeder Setpoint Velocity", self.setpoint_velocity) != self.setpoint_velocity:
            self.setpoint_velocity = wpilib.SmartDashboard.getNumber("SS_Telemetry/Feeder Setpoint Velocity", self.setpoint_velocity)
            # place to update any commands that rely on setpoint_velocity if needed
        if self.setpoint_velocity > 110: # max rps 116.67
            self.setpoint_velocity = 110
            wpilib.SmartDashboard.putNumber("SS_Telemetry/Feeder Setpoint Velocity", self.setpoint_velocity)
        self.current_velocity = self.motor.get_rotor_velocity().value
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Feeder Current Velocity ", self.current_velocity)
        dashboard_p = wpilib.SmartDashboard.getNumber("PIDF/Feeder P", self.P)
        dashboard_i = wpilib.SmartDashboard.getNumber("PIDF/Feeder I", self.I)
        dashboard_d = wpilib.SmartDashboard.getNumber("PIDF/Feeder D", self.D)
        dashboard_ff = wpilib.SmartDashboard.getNumber("PIDF/Feeder FF", self.FF)
        dashboard_s = wpilib.SmartDashboard.getNumber("PIDF/Feeder S", self.S)

        if (
            dashboard_p != self.P
            or dashboard_i != self.I
            or dashboard_d != self.D
            or dashboard_ff != self.FF
            or dashboard_s != self.S
        ):
            self.P = dashboard_p
            self.I = dashboard_i
            self.D = dashboard_d
            self.FF = dashboard_ff
            self.S = dashboard_s
            self._apply_pidf_to_config()

            status = self.motor.configurator.apply(self._config)
            if not status.is_ok():
                wpilib.reportError(f"Kraken PID update failed: {status}", False)

    # -------------------------
    # Motor movement functions
    # -------------------------
    def run_velocity_at_setpoint(self) -> None:
        self.motor.set_control(self.velocity_request.with_velocity(self.setpoint_velocity))

    def stop_motor(self):
        self.motor.set(0)

    # -------------------------
    # Commands
    # -------------------------
    def spin_up_and_wait_command(self):
        class SpinUpAndWait_CommDef(commands2.Command):
            def __init__(self, ss_feeder: SS_FeederKraken):
                super().__init__()
                self.ss_feeder = ss_feeder
                self.addRequirements(ss_feeder) # Ensure no other command uses ss_feeder
                self.timer = wpilib.Timer()
                self.velocity_tolerance = 5 # RPS tolerance for considering the feeder "up to speed"

            def initialize(self):
                self.timer.restart()
                self.ss_feeder.run_velocity_at_setpoint() # Spin up
            def execute(self):
                pass # Command is running

            def isFinished(self):
                return abs(self.ss_feeder.current_velocity-self.ss_feeder.setpoint_velocity) < self.velocity_tolerance

            def end(self, interrupted):
                # Keep spinning even if interrupted, since this command is just for waiting until up to speed.
                if interrupted:
                    wpilib.reportWarning("SpinUpAndWait_Command was interrupted before reaching target velocity!", printTrace=False)
                else:
                    wpilib.reportWarning(f"SpinUpAndWait_Command reached target velocity: {self.timer.get():.1f} seconds.", printTrace=False)
                
        spin_up_and_wait_command = SpinUpAndWait_CommDef(self)
        return spin_up_and_wait_command

