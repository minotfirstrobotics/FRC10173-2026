import wpilib
import phoenix6
import commands2
from phoenix6 import CANBus
from wpilib import SmartDashboard
from ntcore import NetworkTableInstance
from pathplannerlib.auto import NamedCommands

class SS_Kraken(commands2.Subsystem):
    def __init__(self, device_id: int, canbus: CANBus, dashboard_name: str, 
                 inverted: bool=False, brake_mode: bool=False,
                 max_rps: int=100, velocity_setpoint: float=0.0, percent_power_setpoint: float=0.0,
                 kp: float=0.0, ki: float=0.0, kd: float=0.0, kv: float=0.0, ks: float=0.0,
                 ka: float=0.0, kg: float=0.0, vmax: float=0.0, amax: float=0.0, jerk: float=0.0) -> None:
        self.motor = phoenix6.hardware.TalonFX(device_id, canbus)

        self.dashboard_name = dashboard_name
        self.max_rps = max_rps
        self.percent_power_setpoint = percent_power_setpoint
        self.velocity_setpoint = velocity_setpoint
        self.velocity_actual = 0.0
        self._setup_hardware_configuration(inverted, brake_mode)
        self._apply_pidf_to_config(kp, ki, kd, kv, ks, ka, kg, vmax, amax, jerk)
        self.position_setpoint = 0.0
        self.position_actual = 0.0
        self._periodic_counter = 0
        self.motor.set_position(self.position_setpoint) # pidf reset encoder position to 0 on startup

        self._put_telemetry_on_dashboard()
        self._construct_register_and_dashboard_commands()

    def _setup_hardware_configuration(self, inverted: bool=False, brake_mode: bool=False):
        self._config = phoenix6.configs.TalonFXConfiguration()
        if brake_mode:
            self._config.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE
        else:
            self._config.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.COAST
        if inverted:
            self._config.motor_output.inverted = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        else:
            self._config.motor_output.inverted = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE

        self._config.current_limits.stator_current_limit_enable = True
        self._config.current_limits.stator_current_limit = 100.0
        self._config.current_limits.supply_current_limit_enable = True
        self._config.current_limits.supply_current_limit = 40.0

        self.velocity_request = phoenix6.controls.VelocityDutyCycle(0.0)
        self.velocity_FOC_request = phoenix6.controls.VelocityTorqueCurrentFOC(0.0) # advanced control unused
        self.position_request_with_trapezoid = phoenix6.controls.MotionMagicDutyCycle(0.0)

    def _apply_pidf_to_config(self, new_p, new_i, new_d, new_v, new_s, 
                              new_a, new_g, new_vmax, new_amax, new_jerk):
        self._config.slot0.k_p = self.kP = new_p
        self._config.slot0.k_i = self.kI = new_i
        self._config.slot0.k_d = self.kD = new_d
        self._config.slot0.k_v = self.kV = new_v # optional velocity feedforward for running at certain speeds
        self._config.slot0.k_s = self.kS = new_s # optional static feedforward for overcoming static friction
        self._config.slot0.k_a = self.kA = new_a # optional acceleration feedforward for compensating inertia
        self._config.slot0.k_g = self.kG = new_g # optional gravity feedforward for compensating gravity effects
        self.vmax = new_vmax
        self.amax = new_amax
        self.jerk = new_jerk
        if self.vmax or self.amax or self.jerk:
            self._config.motion_magic.motion_magic_cruise_velocity = self.vmax
            self._config.motion_magic.motion_magic_acceleration = self.amax
            self._config.motion_magic.motion_magic_jerk = self.jerk

        self.status = self.motor.configurator.apply(self._config)
        if self.status.is_ok():
            wpilib.reportWarning(f"Applying PIDF config to {self.dashboard_name}: kP={new_p}, kI={new_i}, kD={new_d}, kV={new_v}, kS={new_s}, kA={new_a}, kG={new_g}, vmax={new_vmax}, amax={new_amax}, jerk={new_jerk}", printTrace=False)
        else:
            wpilib.reportError(f"Kraken PID update failed: {self.status}", False)
        SmartDashboard.putBoolean(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Config Success", self.status.is_ok())

    def _put_telemetry_on_dashboard(self):
        SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Velocity Actual", round(self.velocity_actual, 1))
        SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Velocity Setpoint", self.velocity_setpoint)
        SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Power Percent Setpoint", self.percent_power_setpoint)
        SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Position Actual", round(self.position_actual, 2))
        SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Position Setpoint", self.position_setpoint)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kP", self.kP)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kI", self.kI)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kD", self.kD)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kV", self.kV)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kS", self.kS)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} ka", self.kA)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kg", self.kG)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} vmax", self.vmax)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} amax", self.amax)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} jerk", self.jerk)
        nt_pidf = NetworkTableInstance.getDefault().getTable(f"PIDF/{self.dashboard_name}")
        self._nt_kP = nt_pidf.getEntry(f"{self.dashboard_name} kP")
        self._nt_kI = nt_pidf.getEntry(f"{self.dashboard_name} kI")
        self._nt_kD = nt_pidf.getEntry(f"{self.dashboard_name} kD")
        self._nt_kV = nt_pidf.getEntry(f"{self.dashboard_name} kV")
        self._nt_kS = nt_pidf.getEntry(f"{self.dashboard_name} kS")
        self._nt_kA = nt_pidf.getEntry(f"{self.dashboard_name} kA")
        self._nt_kG = nt_pidf.getEntry(f"{self.dashboard_name} kG")
        self._nt_vmax = nt_pidf.getEntry(f"{self.dashboard_name} vmax")
        self._nt_amax = nt_pidf.getEntry(f"{self.dashboard_name} amax")
        self._nt_jerk = nt_pidf.getEntry(f"{self.dashboard_name} jerk")

    def _construct_register_and_dashboard_commands(self):
        self.stop_motor_command = self.stop_motor()
        NamedCommands.registerCommand(f"{self.dashboard_name} Stop Motor", self.stop_motor_command)
        SmartDashboard.putData(f"Commands/{self.dashboard_name}/{self.dashboard_name} Stop Motor", self.stop_motor_command)

        self.run_at_velocity_command = self.run_at_velocity()
        NamedCommands.registerCommand(f"{self.dashboard_name} Run at Setpoint Velocity", self.run_at_velocity_command)
        SmartDashboard.putData(f"Commands/{self.dashboard_name}/{self.dashboard_name} Run at Velocity", self.run_at_velocity_command)

        self.run_power_percent_forward_command = self.run_power_percent_forward()
        NamedCommands.registerCommand(f"{self.dashboard_name} Power Percent Forward", self.run_power_percent_forward_command)
        SmartDashboard.putData(f"Commands/{self.dashboard_name}/{self.dashboard_name} Power Percent Forward", self.run_power_percent_forward_command)

        self.run_power_percent_reverse_command = self.run_power_percent_reverse()
        NamedCommands.registerCommand(f"{self.dashboard_name} Power Percent Reverse", self.run_power_percent_reverse_command)
        SmartDashboard.putData(f"Commands/{self.dashboard_name}/{self.dashboard_name} Power Percent Reverse", self.run_power_percent_reverse_command)
        
        self.spin_up_and_wait_command = self.spin_up_and_wait()
        NamedCommands.registerCommand(f"{self.dashboard_name}/{self.dashboard_name} Spin-up to Setpoint and Wait", self.spin_up_and_wait_command)
        SmartDashboard.putData(f"Commands/{self.dashboard_name}/{self.dashboard_name} Spin-up to Setpoint and Wait", self.spin_up_and_wait_command)

        self.rotate_to_position_command = self.rotate_to_position()
        NamedCommands.registerCommand(f"{self.dashboard_name} Rotate to Position", self.rotate_to_position_command)
        SmartDashboard.putData(f"Commands/{self.dashboard_name}/{self.dashboard_name} Rotate to Position", self.rotate_to_position_command)

        self.rotate_to_position_and_wait_command = self.rotate_to_position_and_wait()
        NamedCommands.registerCommand(f"{self.dashboard_name} Rotate to and Wait", self.rotate_to_position_and_wait_command)
        SmartDashboard.putData(f"Commands/{self.dashboard_name}/{self.dashboard_name} Rotate to Position and Wait", self.rotate_to_position_and_wait_command)

    # -------------------------
    # Periodic tasks - dashboard updates and config changes
    # -------------------------
    def periodic(self):
        self.position_actual = self.motor.get_position().value
        SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Position Actual", round(self.position_actual, 2))
        self.velocity_actual = self.motor.get_velocity().value
        SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Velocity Actual", round(self.velocity_actual, 1))

        self._periodic_counter += 1
        if self._periodic_counter % 5 == 0:  # Every 100ms instead of every 20ms
            dashboard_velocity_setpoint = SmartDashboard.getNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Velocity Setpoint", self.velocity_setpoint)
            if dashboard_velocity_setpoint != self.velocity_setpoint:
                self.velocity_setpoint = max(min(dashboard_velocity_setpoint, self.max_rps), -self.max_rps)
                SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Velocity Setpoint", self.velocity_setpoint)
            dashboard_power_percent_setpoint = SmartDashboard.getNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Power Percent Setpoint", self.percent_power_setpoint)
            if dashboard_power_percent_setpoint != self.percent_power_setpoint:
                self.percent_power_setpoint = max(min(dashboard_power_percent_setpoint, 1.0), -1.0)
                SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Power Percent Setpoint", self.percent_power_setpoint)
            dashboard_position_setpoint = SmartDashboard.getNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Position Setpoint", self.percent_power_setpoint)
            if dashboard_position_setpoint != self.position_setpoint:
                self.position_setpoint = max(min(dashboard_position_setpoint, 1.0), -1.0)
                SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Position Setpoint", self.position_setpoint)
            dashboard_p = self._nt_kP.getDouble(self.kP)
            dashboard_i = self._nt_kI.getDouble(self.kI)
            dashboard_d = self._nt_kD.getDouble(self.kD)
            dashboard_v = self._nt_kV.getDouble(self.kV)
            dashboard_s = self._nt_kS.getDouble(self.kS)
            dashboard_a = self._nt_kA.getDouble(self.kA)
            dashboard_g = self._nt_kG.getDouble(self.kG)
            dashboard_vmax = self._nt_vmax.getDouble(self.vmax)
            dashboard_amax = self._nt_amax.getDouble(self.amax)
            dashboard_jerk = self._nt_jerk.getDouble(self.jerk)
            if (dashboard_p != self.kP or dashboard_i != self.kI or dashboard_d != self.kD or 
                dashboard_v != self.kV or dashboard_s != self.kS or dashboard_a != self.kA or dashboard_g != self.kG or
                dashboard_vmax != self.vmax or dashboard_amax != self.amax or dashboard_jerk != self.jerk):
                self._apply_pidf_to_config(dashboard_p, dashboard_i, dashboard_d, dashboard_v, dashboard_s, 
                                        dashboard_a, dashboard_g, dashboard_vmax, dashboard_amax, dashboard_jerk)
            # self._put_telemetry_on_dashboard()

    # -------------------------
    # Motor movement functions
    # -------------------------
    def _stop_motor(self):
        self.motor.set(0.0)
        # self.motor.stopMotor()

    def _run_at_velocity(self, setpoint = None) -> None:
        if setpoint is None:
            setpoint = self.velocity_setpoint
        """Needs PIDF settings and max_rps to be configured appropriately for good performance."""
        self.motor.set_control(self.velocity_request.with_velocity(setpoint))

    def _run_power_percent(self, setpoint = None) -> None:
        if setpoint is None:
            setpoint = self.velocity_setpoint
        self.motor.set(setpoint)

    def _rotate_to_position(self, target_rotations = None) -> None:
        if target_rotations is None:
            target_rotations = self.position_setpoint
        self.motor.set_control(self.position_request_with_trapezoid.with_position(float(target_rotations)))
        self.position_setpoint = target_rotations

    # -------------------------
    # Commands
    # -------------------------
    def stop_motor(self):
        return commands2.cmd.runOnce(lambda: self._stop_motor(), self)

    def run_at_velocity(self, setpoint = None):
        if setpoint is None:
            setpoint = self.velocity_setpoint
        return commands2.cmd.runOnce(lambda: self._run_at_velocity(setpoint), self)
    
    def run_power_percent_forward(self, setpoint = None):
        if setpoint is None:
            setpoint = self.percent_power_setpoint
        return commands2.cmd.runOnce(lambda: self._run_power_percent(setpoint), self)
    
    def run_power_percent_reverse(self, setpoint = None):
        if setpoint is None:
            setpoint = self.percent_power_setpoint
        return commands2.cmd.runOnce(lambda: self._run_power_percent(-setpoint), self)

    def rotate_to_position(self, target_rotations = None):
        if target_rotations is None:
            target_rotations = self.position_setpoint
        return commands2.cmd.runOnce(lambda: self._rotate_to_position(target_rotations), self)

    def spin_up_and_wait(self):
        return commands2.FunctionalCommand(
            lambda: self._run_at_velocity(),
            lambda: None,
            lambda interrupted: wpilib.reportWarning(f"SpinUpAndWait_Command {'interrupted before reaching target velocity!' if interrupted else 'reached target velocity!'}", printTrace=False),
            lambda: abs(self.velocity_actual - self.velocity_setpoint) < 10,
            self )

    def rotate_to_position_and_wait(self, target_rotations = None):
        if target_rotations is None:
            target_rotations = self.position_setpoint
        return commands2.FunctionalCommand(
            lambda: self._rotate_to_position(target_rotations),
            lambda: None,
            lambda: wpilib.reportWarning(f"reached target position", printTrace=False),
            lambda: abs(self.position_actual - target_rotations) < 0.1,
            self )

    def spin_up_and_wait_verbose(self):
        class SpinUpAndWait(commands2.Command):
            def __init__(self, ss_kraken: SS_Kraken):
                super().__init__()
                self.ss_kraken = ss_kraken
                self.addRequirements(ss_kraken) # Ensure no other command uses subsystems
                self.timer = wpilib.Timer()
                self.velocity_tolerance = 10 # RPS tolerance for considering the kraken "up to speed"

            def initialize(self):
                self.timer.restart()
                self.ss_kraken._run_at_velocity() # Spin up

            def execute(self):
                pass # Command is running

            def isFinished(self):
                return abs(self.ss_kraken.velocity_actual - self.ss_kraken.velocity_setpoint) < self.velocity_tolerance

            def end(self, interrupted):
                # Keep spinning even if interrupted, since this command is just for waiting until up to speed.
                if interrupted:
                    wpilib.reportWarning("SpinUpAndWait_Command was interrupted before reaching target velocity!", printTrace=False)
                else:
                    wpilib.reportWarning(f"SpinUpAndWait_Command reached target velocity: {self.timer.get():.1f} seconds.", printTrace=False)
                
        spin_up_and_wait_command = SpinUpAndWait(self)
        return spin_up_and_wait_command


