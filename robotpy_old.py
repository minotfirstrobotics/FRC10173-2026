import commands2
import wpilib
from wpilib import SmartDashboard, Timer
from wpimath.geometry import Pose2d, Rotation2d
from phoenix6 import HootAutoReplay
from pathplannerlib.auto import AutoBuilder, NamedCommands
from commands2.button import CommandXboxController
from subsystems.SS_SwerveDrive import SS_SwerveDrive
#from subsystems.SS_ShooterNEO import SS_ShooterNEO
#from subsystems.SS_TurretTalon_Trapezoidal import SS_TurretTalon
#from subsystems.SS_FeederTalon_Power import SS_FeederTalon_Power
#from subsystems.SS_IntakeSIMM import SS_IntakeSIMM
#from subsystems.SS_CANdleLight import SS_CANdleLight
#from subsystems.SS_CameraPose import SS_CameraPose
#from commands.CMD_ComboShoot import CMD_ComboShoot
#from commands.SEQ_Shoot import SEQ_Shoot


class RobotContainer:
    def __init__(self) -> None:
        self.joystick = CommandXboxController(0)
        self.ss_swerve_drive = SS_SwerveDrive(self.joystick)
        #self.ss_shooter_spark = SS_ShooterNEO(3, self.joystick)
        #self.ss_turret_talon = SS_TurretTalon(1, self.joystick)
        #self.ss_feeder_talon = SS_FeederTalon_Power(0, self.joystick)
        #self.ss_intake_spark = SS_IntakeSIMM(4, self.joystick)
        #self.ss_candle_light_rear = SS_CANdleLight(2)
        #self.ss_candle_light_front = SS_CANdleLight(5)
        # self.ss_camera_pose = SS_CameraPose(self.ss_swerve_drive)

        # Build auto chooser from PathPlanner auto.auto files in deploy folder
        self.auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Autonomous Routine", self.auto_chooser)

        # ss_shoot_command = CMD_ComboShoot(self.ss_shooter_spark, self.ss_feeder_talon, self.joystick)
        # self.joystick.rightBumper().onTrue(ss_shoot_command)

        # self.shoot_balls_sequence = SEQ_Shoot(self.ss_shooter_spark, self.ss_feeder_talon)
        # self.joystick.rightBumper().onTrue(self.shoot_balls_sequence)
        # self.joystick.rightBumper().onFalse(self.ss_shooter_spark.stop_motor_command())
        # self.joystick.rightBumper().onFalse(self.ss_feeder_talon.stop_motor_command())

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
        # self._time_and_joystick_replay = (HootAutoReplay().with_timestamp_replay().with_joystick_replay() )
    
    def autonomousInit(self) -> None:
        self.localMatchTimer.reset()
        self.localMatchTimer.start()
    
        whatauto = self.container.getAutonomousCommand()
    
        # grab autos
        if callable(whatauto):
            try:
                whatauto = whatauto()
            except Exception as e:
                print(f"Auto factory failed: {e}")
                return
    
        # validate
        if not hasattr(whatauto, "schedule"):
            print(f"Invalid autonomous command: {whatauto!r}")
            return
    
        # schedule
        try:
            whatauto.schedule()
        except Exception as e:
            print(f"Scheduling failed: {e}")

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
        # self._time_and_joystick_replay.update() # using HootAutoReplay to log and replay timestamp and joystick data
 
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