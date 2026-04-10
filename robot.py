import wpilib
import commands2
from commands2 import cmd
from commands2.button import Trigger
from wpilib import Color8Bit, SmartDashboard, Timer, DriverStation
from phoenix6 import HootAutoReplay
from pathplannerlib.auto import AutoBuilder, NamedCommands
from commands2.button import CommandXboxController
from generated.tuner_constants_2026_GF import TunerConstants
from subsystems.SS_SwerveDrive import SS_SwerveDrive, PathfindPOVTarget
from subsystems.SS_Kraken import SS_Kraken
from subsystems.SS_CANdleLight import SS_CANdleLight
from subsystems.SS_CameraPose_left_old import SS_CameraPose_Left
from subsystems.SS_CameraPose_right_old import SS_CameraPose_Right
from commands.complex_and_sequences import CMD_ComboShoot, SEQ_shoot
from commands.auto_distance_shoot import CMD_AutoDistanceShoot

class RobotContainer:
    def __init__(self) -> None:
        self.gamepad = None or CommandXboxController(0)
        DriverStation.silenceJoystickConnectionWarning(True)
        self.canbus = TunerConstants.canbus
        self.ss_shooter = None or SS_Kraken(3, self.canbus, "Shooter", inverted=True, max_rps=100, velocity_setpoint=40, kp=0.08, ki=0.0, kd=0.0, kv=0.012, ks=0.0)
        self.ss_feeder = None or SS_Kraken(1, self.canbus, "Feeder", kp=1.0, velocity_setpoint=10, percent_power_setpoint=0.5)
        self.ss_intake = None or SS_Kraken(4, self.canbus, "Intake", max_rps=120, percent_power_setpoint=0.55)
        self.ss_candle_light_left = None or SS_CANdleLight(2, self.canbus, "Left")
        self.ss_candle_light_right = None or SS_CANdleLight(5, self.canbus, "Right")
        self.ss_swerve_drive = None or SS_SwerveDrive(self.gamepad)
        self.ss_camera_pose_left = None or SS_CameraPose_Left(self.ss_swerve_drive)
        self.ss_camera_pose_right = None or SS_CameraPose_Right(self.ss_swerve_drive)

        self._build_complex_commands_and_autochooser()
        self._setup_simulated_mechanism2d()
        self.defaultdrivemode = self.ss_swerve_drive.drive_mode_field_centered()

        if self.gamepad: self.configure_gamepad_bindings()

    def configure_gamepad_bindings(self):
        self.ss_swerve_drive.setDefaultCommand(self.defaultdrivemode)
        if self.ss_shooter:
            self.gamepad.rightBumper().whileTrue(self.ss_shooter.hold_dashboard_velocity())
        if self.ss_feeder:
            self.gamepad.leftBumper().whileTrue(self.ss_feeder.hold_dashboard_velocity())
        if self.ss_intake:
            self.gamepad.leftTrigger(threshold=.2).whileTrue(self.ss_intake.hold_dashboard_power_percent(-1.0))
            self.gamepad.rightTrigger(threshold=.2).whileTrue(self.ss_intake.hold_dashboard_power_percent(1.0))
        self.cmd_combo_shoot = CMD_ComboShoot(self.ss_shooter, self.ss_feeder, self.ss_swerve_drive)
        self.gamepad.x().whileTrue(self.cmd_combo_shoot)


        if self.ss_swerve_drive:
            self.gamepad.a().onTrue(self.ss_swerve_drive.drive_mode_padlocked())
            self.gamepad.a().onFalse(self.defaultdrivemode)

            Trigger(lambda: self.gamepad.getHID().getPOV() == PathfindPOVTarget.TOP_RIGHT).onTrue(
                self.ss_swerve_drive.pathfind_to_pov_zone(PathfindPOVTarget.TOP_RIGHT)
            )
            Trigger(lambda: self.gamepad.getHID().getPOV() == PathfindPOVTarget.BOTTOM_RIGHT).onTrue(
                self.ss_swerve_drive.pathfind_to_pov_zone(PathfindPOVTarget.BOTTOM_RIGHT)
            )
            Trigger(lambda: self.gamepad.getHID().getPOV() == PathfindPOVTarget.BOTTOM_LEFT).onTrue(
                self.ss_swerve_drive.pathfind_to_pov_zone(PathfindPOVTarget.BOTTOM_LEFT)
            )
            Trigger(lambda: self.gamepad.getHID().getPOV() == PathfindPOVTarget.TOP_LEFT).onTrue(
                self.ss_swerve_drive.pathfind_to_pov_zone(PathfindPOVTarget.TOP_LEFT)
            )

            self.gamepad.back().and_(self.gamepad.start()).onTrue(
                cmd.runOnce(self.ss_swerve_drive.reset_field_oriented_perspective) )

    def _build_complex_commands_and_autochooser(self):
        self.cmd_combo_shoot = CMD_ComboShoot(self.ss_shooter, self.ss_feeder, self.ss_swerve_drive)
        SmartDashboard.putData("Commands/Combo Shoot", self.cmd_combo_shoot)
        NamedCommands.registerCommand("Combo Shoot", self.cmd_combo_shoot)

        self.seq_shoot = SEQ_shoot(self.ss_shooter, self.ss_feeder)
        SmartDashboard.putData("Commands/SEQ Shoot", self.seq_shoot)
        NamedCommands.registerCommand("SEQ Shoot", self.seq_shoot)

        self.run_intake = self.ss_intake.run_power_percent_forward_dashboard()
        SmartDashboard.putData("Commands/Run Intake", self.run_intake)
        NamedCommands.registerCommand("Run Intake", self.run_intake)

        self.auto_distance_shoot_command = CMD_AutoDistanceShoot(self.ss_shooter, self.ss_swerve_drive)

        self.unload = cmd.sequence(
            self.ss_feeder.hold_velocity(self.ss_feeder.get_dashboard_velocity_setpoint).withTimeout(1.0),
            self.ss_feeder.stop_motor(),
        )
        SmartDashboard.putData("Commands/Unload", self.unload)
        NamedCommands.registerCommand("Unload", self.unload)
        
        self.auto_chooser = AutoBuilder.buildAutoChooser("None") # must be defined after SS's and all registered commands
        SmartDashboard.putData("Auto Chooser", self.auto_chooser)

    def _setup_simulated_mechanism2d(self):
        self.mech2d = wpilib.Mechanism2d(10, 10)  # Width, Height
        self.root2d = self.mech2d.getRoot("root", 5, 2)
        if self.ss_intake:
            self.intake2d = self.root2d.appendLigament("intake", 4, 45, 6, Color8Bit(0, 0, 255))
        if self.ss_feeder:
            self.feeder2d = self.root2d.appendLigament("feeder", 4, 0, 6, Color8Bit(0, 255, 0))
        if self.ss_shooter:
            self.shooter2d = self.root2d.appendLigament("shooter", 4, 135, 6, Color8Bit(255, 0, 0))
        wpilib.SmartDashboard.putData("Mechanism", self.mech2d)

    def get_autonomous_command(self) -> commands2.Command:
        return self.auto_chooser.getSelected()


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """
    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.

        Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        autonomous chooser on the dashboard.
        """
        self.autonomousCommand = None
        self.container = RobotContainer()
        self.localMatchTimer = Timer()
        # self._time_and_driver_replay = (HootAutoReplay().with_timestamp_replay().with_driver_replay() )
        if self.container.ss_candle_light_right:
            self.container.ss_candle_light_right.set_all_leds_RGBW(0, 255, 0) # Set front CANdle to green
        if self.container.ss_candle_light_left:
            self.container.ss_candle_light_left.set_all_leds_RGBW(255, 165, 0) # Set rear CANdle to orange

    def robotPeriodic(self) -> None:
        """
        This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating.

        Starts by running the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        commands, running already-scheduled commands, removing finished or interrupted commands,
        and running subsystem periodic() methods.  This must be called from the robot's periodic
        block in order for anything in the Command-based framework to work.
        """
        # self._time_and_driver_replay.update() # using HootAutoReplay to log and replay timestamp and driver data
        match_time_from_driver_station = Timer.getMatchTime()
        SmartDashboard.putNumber("Match Time", self.localMatchTimer.get() if match_time_from_driver_station < 0 else match_time_from_driver_station)
        voltage = wpilib.RobotController.getBatteryVoltage()
        SmartDashboard.putNumber("Battery Voltage", voltage)
        if self.container.ss_intake:
            self.container.intake2d.setLength(-self.container.ss_intake.velocity_actual/10)
        if self.container.ss_feeder:
            self.container.feeder2d.setLength(self.container.ss_feeder.velocity_actual/10)
        if self.container.ss_shooter:
            self.container.shooter2d.setLength(self.container.ss_shooter.velocity_actual/10)
            calculated_shooter_speed = self.container.auto_distance_shoot_command.get_required_shooter_speed()
            SmartDashboard.putNumber("SS_Telemetry/Shooter/Shooter Auto Distance Speed", calculated_shooter_speed)


    def teleopInit(self) -> None:
        """
        This makes sure that the autonomous stops running when
        teleop starts running. If you want the autonomous to
        continue until interrupted by another command, remove
        this line or comment it out. 
        """
        self.container.ss_swerve_drive.setDefaultCommand(
            self.container.defaultdrivemode
        )

        self.localMatchTimer.reset()
        self.localMatchTimer.start()
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            if self.container.ss_candle_light_right:
                self.container.ss_candle_light_right.set_all_leds_RGBW(255, 0, 0)
            if self.container.ss_candle_light_left:
                self.container.ss_candle_light_left.set_all_leds_RGBW(255, 0, 0)
        elif DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            if self.container.ss_candle_light_right:
                self.container.ss_candle_light_right.set_all_leds_RGBW(0, 0, 255)
            if self.container.ss_candle_light_left:
                self.container.ss_candle_light_left.set_all_leds_RGBW(0, 0, 255)

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass


    def autonomousInit(self) -> None:
        """
        This function is run once when the robot enters autonomous mode.
        Gets the selected autonomous command from the chooser and schedules it.
        """
        self.container.ss_swerve_drive.removeDefaultCommand()
        self.localMatchTimer.reset()
        self.localMatchTimer.start()
        self.container.ss_swerve_drive.reset_field_oriented_perspective()
        self.autonomousCommand = self.container.get_autonomous_command()
        
        if not self.autonomousCommand:
            print("No autonomous command selected.")
            return

        self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        pass


    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        pass


    def testInit(self) -> None:
        """Cancels all running commands at the start of test mode"""
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self) -> None:
        """This function is called periodically during test mode"""
        pass

    def testExit(self) -> None:
        """This function is called once when exiting test mode."""
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)
