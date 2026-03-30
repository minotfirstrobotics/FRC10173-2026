import wpilib
import phoenix6
import commands2
from phoenix6 import CANBus
from pathplannerlib.auto import NamedCommands

class SS_Kraken(commands2.Subsystem):
    def __init__(self, device_id: int, canbus: CANBus, dashboard_name: str, 
                 inverted: bool=False, brake_mode: bool=False,
                 max_rps: int=100, velocity_setpoint: float=0.0, percent_power_setpoint: float=0.62,
                 kp: float=0.01, ki: float=0.0, kd: float=0.0, kv: float=0.0, ks: float=0.0) -> None:
        super().__init__()
        self.motor = phoenix6.hardware.TalonFX(device_id, canbus)
        self._config = phoenix6.configs.TalonFXConfiguration()
        if brake_mode:
            self._config.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE
        else:
            self._config.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.COAST
        if inverted:
            self._config.motor_output.inverted = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        else:
            self._config.motor_output.inverted = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
        self.velocity_request = phoenix6.controls.VelocityDutyCycle(0.0)
        self.voltage_request  = phoenix6.controls.VoltageOut(0.0)

        self._config.current_limits.stator_current_limit_enable = True
        self._config.current_limits.stator_current_limit = 80.0
        self._config.current_limits.supply_current_limit_enable = True
        self._config.current_limits.supply_current_limit = 40.0

        self.dashboard_name = dashboard_name
        self.max_rps = max_rps
        self.percent_power_setpoint = percent_power_setpoint
        self.velocity_setpoint = velocity_setpoint
        self.velocity_actual = 0.0
        self._apply_pidf_to_config(kp, ki, kd, kv, ks)

        wpilib.SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name} Velocity Actual", self.velocity_actual)
        wpilib.SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name} Velocity Setpoint", self.velocity_setpoint)
        wpilib.SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name} Power Percent Setpoint", self.percent_power_setpoint)
        wpilib.SmartDashboard.putNumber(f"PIDF/{dashboard_name} kP", self.kP)
        wpilib.SmartDashboard.putNumber(f"PIDF/{dashboard_name} kI", self.kI)
        wpilib.SmartDashboard.putNumber(f"PIDF/{dashboard_name} kD", self.kD)
        wpilib.SmartDashboard.putNumber(f"PIDF/{dashboard_name} kV", self.kV)
        wpilib.SmartDashboard.putNumber(f"PIDF/{dashboard_name} kS", self.kS)

        # NamedCommands.registerCommand(f"{self.dashboard_name} Spin-up to Setpoint", self.spin_up_and_wait_command())
        NamedCommands.registerCommand(f"{self.dashboard_name} Power Percent Forward", commands2.cmd.runOnce(self.run_voltage_percent_forward))
        NamedCommands.registerCommand(f"{self.dashboard_name} Power Percent Reverse", commands2.cmd.runOnce(self.run_voltage_percent_reverse))
        NamedCommands.registerCommand(f"{self.dashboard_name} Velocity Setpoint", commands2.cmd.runOnce(self.run_velocity_at_setpoint))
        NamedCommands.registerCommand(f"{self.dashboard_name} Stop", commands2.cmd.runOnce(self.stop_motor))

    def _apply_pidf_to_config(self, dashboard_p, dashboard_i, dashboard_d, dashboard_v, dashboard_s):
        self.kP = dashboard_p
        self.kI = dashboard_i
        self.kD = dashboard_d
        self.kV = dashboard_v
        self.kS = dashboard_s
        self._config.slot0.k_p = self.kP
        self._config.slot0.k_i = self.kI
        self._config.slot0.k_d = self.kD
        self._config.slot0.k_v = self.kV
        self._config.slot0.k_s = self.kS

        self.status = self.motor.configurator.apply(self._config)
        if not self.status.is_ok():
            wpilib.reportError(f"Kraken PID update failed: {self.status}", False)
        wpilib.SmartDashboard.putBoolean(f"SS_Telemetry/{self.dashboard_name} Config Success", self.status.is_ok())

    # -------------------------
    # Periodic tasks - dashboard updates and config changes
    # -------------------------
    def periodic(self):
        self.velocity_actual = self.motor.get_velocity().value
        wpilib.SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name} Velocity Actual", self.velocity_actual)
        dashboard_velocity_setpoint = wpilib.SmartDashboard.getNumber(f"SS_Telemetry/{self.dashboard_name} Velocity Setpoint", self.velocity_setpoint)
        if dashboard_velocity_setpoint != self.velocity_setpoint:
            self.velocity_setpoint = min(dashboard_velocity_setpoint, self.max_rps)
            wpilib.SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name} Velocity Setpoint", self.velocity_setpoint)
        dashboard_power_percent_setpoint = wpilib.SmartDashboard.getNumber(f"SS_Telemetry/{self.dashboard_name} Power Percent Setpoint", self.percent_power_setpoint)
        if dashboard_power_percent_setpoint != self.percent_power_setpoint:
            self.percent_power_setpoint = max(min(dashboard_power_percent_setpoint, 1.0), -1.0)
            wpilib.SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name} Power Percent Setpoint", self.percent_power_setpoint)

        dashboard_p =  wpilib.SmartDashboard.getNumber(f"PIDF/{self.dashboard_name} kP", self.kP)
        dashboard_i =  wpilib.SmartDashboard.getNumber(f"PIDF/{self.dashboard_name} kI", self.kI)
        dashboard_d =  wpilib.SmartDashboard.getNumber(f"PIDF/{self.dashboard_name} kD", self.kD)
        dashboard_ff = wpilib.SmartDashboard.getNumber(f"PIDF/{self.dashboard_name} kV", self.kV)
        dashboard_s =  wpilib.SmartDashboard.getNumber(f"PIDF/{self.dashboard_name} kS", self.kS)
        if (dashboard_p != self.kP or dashboard_i != self.kI or dashboard_d != self.kD or dashboard_ff != self.kV or dashboard_s != self.kS):
            self._apply_pidf_to_config(dashboard_p, dashboard_i, dashboard_d, dashboard_ff, dashboard_s)

    # -------------------------
    # Motor movement functions
    # -------------------------
    def run_velocity_at_setpoint(self) -> None:
        """Needs PIDF settings and max_rps to be configured appropriately for good performance."""
        self.motor.set_control(self.velocity_request.with_velocity(self.velocity_setpoint))

    def run_voltage_percent_forward(self) -> None:
        """Does not need PIDF settings"""
        self.motor.set(self.percent_power_setpoint)

    def run_voltage_percent_reverse(self) -> None:
        """Does not need PIDF settings"""
        self.motor.set(-self.percent_power_setpoint)

    def stop_motor(self):
        self.motor.set(0)
        # self.motor.stopMotor()

    # -------------------------
    # Commands
    # -------------------------
    def spin_up_and_wait_command(self):
        class SpinUpAndWait_CommDef(commands2.Command):
            def __init__(self, ss_kraken: SS_Kraken):
                super().__init__()
                self.ss_kraken = ss_kraken
                self.addRequirements(ss_kraken) # Ensure no other command uses subsystems
                self.timer = wpilib.Timer()
                self.velocity_tolerance = 5 # RPS tolerance for considering the {self.dashboard_name} "up to speed"

            def initialize(self):
                self.timer.restart()
                self.ss_kraken.run_velocity_at_setpoint() # Spin up

            def execute(self):
                pass # Command is running

            def isFinished(self):
                return abs(self.ss_kraken.velocity_actual-self.ss_kraken.velocity_setpoint) < self.velocity_tolerance

            def end(self, interrupted):
                # Keep spinning even if interrupted, since this command is just for waiting until up to speed.
                if interrupted:
                    wpilib.reportWarning("SpinUpAndWait_Command was interrupted before reaching target velocity!", printTrace=False)
                else:
                    wpilib.reportWarning(f"SpinUpAndWait_Command reached target velocity: {self.timer.get():.1f} seconds.", printTrace=False)
                
        spin_up_and_wait_command = SpinUpAndWait_CommDef(self)
        return spin_up_and_wait_command

