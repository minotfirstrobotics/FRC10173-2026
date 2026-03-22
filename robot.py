import commands2
import wpilib
from wpilib import SmartDashboard, Timer, DriverStation
from wpimath.geometry import Pose2d, Rotation2d
from phoenix6 import HootAutoReplay
from pathplannerlib.auto import AutoBuilder, NamedCommands
from commands2.button import CommandXboxController
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from subsystems.SS_ShooterKraken import SS_ShooterKraken
from examples.SS_FeederTalon_Power import SS_FeederTalon_Power
from examples.SS_IntakeSIMM import SS_IntakeSIMM
from subsystems.SS_CANdleLight import SS_CANdleLight
from subsystems.SS_CameraPose import SS_CameraPose
from commands.CMD_ComboShoot import CMD_ComboShoot
from commands.SEQ_sequences import SEQ_Shoot, SEQ_DeployIntake


class RobotContainer:
    def __init__(self) -> None:
        self.driver = CommandXboxController(0)
        DriverStation.silenceJoystickConnectionWarning(True)
        # self.operator = CommandXboxController(1)
        self.ss_shooter_kraken = SS_ShooterKraken(3, self.driver)
        self.ss_feeder_talon = SS_FeederTalon_Power(0, self.driver)
        self.ss_intake_spark = SS_IntakeSIMM(4, self.driver)
        #self.ss_candle_light_rear = SS_CANdleLight(2)
        self.ss_candle_light_front = SS_CANdleLight(5)
        self.ss_swerve_drive = SS_SwerveDrive(self.driver)
        self.ss_camera_pose = SS_CameraPose(self.ss_swerve_drive)

        # Build auto chooser from PathPlanner auto.auto files in deploy folder
        self.auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Autonomous Routine", self.auto_chooser)

        # self.configure_driver_inputs()
        # self.configure_operator_inputs()
        # self.configure_inputs_for_testing_commands()

    '''
    def configure_driver_inputs(self):
        self.driver.rightBumper().whileTrue(self.ss_shooter_kraken.run_setpoint_velocity_command())
        self.driver.a().whileTrue(self.ss_intake_spark.run_intake_command())
        self.driver.b().whileTrue(self.ss_feeder_talon.run_outtake_command())
        self.driver.y().whileTrue(self.ss_camera_pose.run_vision_align_command())
        self.driver.x().whileTrue(self.ss_shooter_kraken.run_shooter_spin_command())
        self.driver.leftBumper().whileTrue(self.ss_swerve_drive.field_oriented_command())
        self.driver.leftTrigger(threshold=0.2).whileTrue(self.ss_swerve_drive.precision_mode_command())
        self.driver.rightBumper().whileTrue(self.ss_swerve_drive.robot_oriented_command())
        self.driver.rightTrigger(threshold=0.2).whileTrue(self.ss_swerve_drive.boost_mode_command())
    '''
        
    '''
    def configure_operator_inputs(self):
        self.operator.y().whileTrue(self.run_vision_align_command())
        self.operator.x().whileTrue(self.run_shooter_spin_command())
        self.operator.leftBumper().whileTrue(self.swerve.field_oriented_command())
        self.operator.leftTrigger(threshold=0.2).whileTrue(self.swerve.precision_mode_command())
        self.operator.rightBumper().whileTrue(self.swerve.robot_oriented_command())
        self.operator.rightTrigger(threshold=0.2).whileTrue(self.swerve.boost_mode_command())
        self.operator.povUp().whileTrue(self.swerve.slow_mode_command())
        self.operator.povDown().whileTrue(self.swerve.normal_mode_command())
        self.operator.povLeft().whileTrue(self.swerve.fast_mode_command())
        self.operator.povRight().whileTrue(self.swerve.turbo_mode_command())
    '''

    def configure_inputs_for_testing_commands(self):
        ''' 
        This is where you can bind buttons to commands for testing purposes, 
        without affecting your main driver/operator bindings. 
        '''
        # ss_shoot_command = CMD_ComboShoot(self.ss_shooter_kraken, self.ss_feeder_talon, self.driver)
        # self.driver.rightBumper().onTrue(ss_shoot_command)
        # self.shoot_balls_sequence = SEQ_Shoot(self.ss_shooter_kraken, self.ss_feeder_talon)
        # self.driver.rightBumper().onTrue(self.shoot_balls_sequence)
        # self.driver.rightBumper().onFalse(self.ss_shooter_kraken.stop_motor_command())
        # self.driver.rightBumper().onFalse(self.ss_feeder_talon.stop_motor_command())
        # self.driver.x().onFalse(self.spin_up_and_wait_command())

    def getAutonomousCommand(self) -> commands2.Command:
        """Get the selected autonomous command from the chooser."""
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
        commands2.CommandScheduler.getInstance().run()
        match_time_from_driver_station = Timer.getMatchTime()
        SmartDashboard.putNumber("Match Time", self.localMatchTimer.get() if match_time_from_driver_station < 0 else match_time_from_driver_station)


    def teleopInit(self) -> None:
        """
        This makes sure that the autonomous stops running when
        teleop starts running. If you want the autonomous to
        continue until interrupted by another command, remove
        this line or comment it out. 
        """
        self.localMatchTimer.reset()
        self.localMatchTimer.start()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass


    def autonomousInit(self) -> None:
        """
        This function is run once when the robot enters autonomous mode.
        Gets the selected autonomous command from the chooser and schedules it.
        """
        self.localMatchTimer.reset()
        self.localMatchTimer.start()

        # Get the selection from the chooser. Some chooser implementations
        # may return a callable that constructs a command, or return an
        # object that already is a command. Be defensive and handle both.
        self.autonomousCommand = self.container.getAutonomousCommand()

        # If the chooser returned a factory (callable), call it to get the actual command instance.
        if callable(self.autonomousCommand):
            try:
                self.autonomousCommand = self.autonomousCommand()
            except Exception as e:
                # If the factory raises, log and clear autonomousCommand.
                print(f"Error creating autonomous command from chooser: {e}")
                self.autonomousCommand = None

        # If the selected object has a schedule() method, schedule it.
        if self.autonomousCommand is not None and hasattr(self.autonomousCommand, "schedule"):
            try:
                self.autonomousCommand.schedule()
            except Exception as e:
                print(f"Failed to schedule autonomous command: {e}")
                self.autonomousCommand = None
        else:
            # Helpful debug information when something unexpected is returned
            if self.autonomousCommand is not None:
                print(f"Autonomous selection is not scheduleable: {type(self.autonomousCommand)!r} -> {self.autonomousCommand!r}")

        # Klingbeil's old manual autonomous command scheduling (before PathPlanner auto.auto files and AutoBuilder)
        # auto = SEQ_DeployIntake(self.container.ss_swerve_drive, self.container.ss_shooter_spark, self.container.ss_feeder_talon, self.container.ss_intake_spark)
        # auto.schedule()

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


if __name__ == "__main__":
    wpilib.run(MyRobot)