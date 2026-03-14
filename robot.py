import commands2
import wpilib
from wpilib import SmartDashboard, Timer
from phoenix6 import HootAutoReplay
from pathplannerlib.auto import AutoBuilder, NamedCommands
from commands2.button import CommandXboxController
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from subsystems.SS_ShooterNEO import SS_ShooterNEO
from subsystems.SS_TurretTalon_Trapezoidal import SS_TurretTalon
from subsystems.SS_FeederTalon_Power import SS_FeederTalon_Power
from subsystems.SS_IntakeSIMM import SS_IntakeSIMM
from subsystems.SS_CANdleLight import SS_CANdleLight
from subsystems.SS_CameraPose import SS_CameraPose
from commands.CMD_ComboShoot import CMD_ComboShoot
from commands.SEQ_Shoot import SEQ_Shoot


class RobotContainer:
    def __init__(self) -> None:
        self.driver = CommandXboxController(0)
        # self.operator = CommandXboxController(1)
        self.ss_shooter_spark = SS_ShooterNEO(3, self.driver)
        self.ss_turret_talon = SS_TurretTalon(1, self.driver)
        self.ss_feeder_talon = SS_FeederTalon_Power(0, self.driver)
        self.ss_intake_spark = SS_IntakeSIMM(4, self.driver)
        #self.ss_candle_light_rear = SS_CANdleLight(2)
        self.ss_candle_light_front = SS_CANdleLight(5)
        self.ss_swerve_drive = SS_SwerveDrive(self.driver)
        self.ss_camera_pose = SS_CameraPose(self.ss_swerve_drive)

        # self.auto_chooser = AutoBuilder.buildAutoChooser("Autonomous Mode")
        # SmartDashboard.putData("Default Autonomous", self.auto_chooser)

        # self.configure_driver_inputs()
        # self.configure_operator_inputs()

        # ss_shoot_command = CMD_ComboShoot(self.ss_shooter_spark, self.ss_feeder_talon, self.joystick)
        # self.joystick.rightBumper().onTrue(ss_shoot_command)

        # self.shoot_balls_sequence = SEQ_Shoot(self.ss_shooter_spark, self.ss_feeder_talon)
        # self.joystick.rightBumper().onTrue(self.shoot_balls_sequence)
        # self.joystick.rightBumper().onFalse(self.ss_shooter_spark.stop_motor_command())
        # self.joystick.rightBumper().onFalse(self.ss_feeder_talon.stop_motor_command())


    def configure_driver_inputs(self):
        self.driver.a().whileTrue(self.run_intake_command())
        self.driver.b().whileTrue(self.run_outtake_command())
        self.driver.y().whileTrue(self.run_vision_align_command())
        self.driver.x().whileTrue(self.run_shooter_spin_command())
        self.driver.leftBumper().whileTrue(self.swerve.field_oriented_command())
        self.driver.leftTrigger(threshold=0.2).whileTrue(self.swerve.precision_mode_command())
        self.driver.rightBumper().whileTrue(self.swerve.robot_oriented_command())
        self.driver.rightTrigger(threshold=0.2).whileTrue(self.swerve.boost_mode_command())

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
        commands2.CommandScheduler.getInstance().run()
        # self._time_and_driver_replay.update() # using HootAutoReplay to log and replay timestamp and driver data
 
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime())
        ds_match_time = Timer.getMatchTime()
        SmartDashboard.putNumber("Match Time (DS)", ds_match_time)
        # fallback when DS time unavailable
        if ds_match_time < 0:
            SmartDashboard.putNumber("Match Time", self.localMatchTimer.get())
        else:
            SmartDashboard.putNumber("Match Time", ds_match_time)


    def teleopInit(self) -> None:
        """This makes sure that the autonomous stops running when
        teleop starts running. If you want the autonomous to
        continue until interrupted by another command, remove
        this line or comment it out. """
        self.localMatchTimer.reset()
        self.localMatchTimer.start()
        # if self.autonomousCommand: self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass


    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.localMatchTimer.reset()
        self.localMatchTimer.start()
        # self.autonomousCommand = self.container.auto_chooser.getSelected()
        # if self.autonomousCommand: self.autonomousCommand.schedule()

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