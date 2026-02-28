import commands2
import constants
from wpimath.units import rotationsToRadians
from phoenix6 import swerve, SignalLogger
from telemetry import Telemetry
from generated.tuner_constants import TunerConstants
from wpilib import DriverStation, Timer, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d
from commands2.button import Trigger
from commands2.sysid import SysIdRoutine

class SS_SwerveDrive(commands2.Subsystem):
    def __init__(self, joystick) -> None:
        super().__init__()
        self._joystick = joystick
        self._max_angular_rate = rotationsToRadians(constants.SWERVE_DEFAULT_NOT_GENERATED["MAX_ROTATION_SPEED"]) # .75 was recommended
        self._max_speed = constants.SWERVE_DEFAULT_NOT_GENERATED["MAX_DRIVE_SPEED_FACTOR"] * TunerConstants.speed_at_12_volts
        self._pov_speed = constants.SWERVE_DEFAULT_NOT_GENERATED["MAX_POV_SPEED"]
        self._latest_pose = Pose2d()
        self._logger = Telemetry(self._max_speed)
        self.drivetrain = TunerConstants.create_drivetrain() # does this need to after swerve configs?
        self.PIDF_sysID_tuning_bindings()
        idle = swerve.requests.Idle() # Determine behavior when no other commands are running. 
        Trigger(DriverStation.isDisabled).whileTrue( # This is important to prevent unexpected robot movement when commands end.
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        # Initialize swerve drive configurations
        self._drive_field_centered = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.05)
            .with_rotational_deadband(self._max_angular_rate * 0.05)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self._drive_facing_direction = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._max_speed * 0.05)
            .with_rotational_deadband(self._max_angular_rate * 0.05)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self._drive_robot_centered = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self.heading_is_driver_controlled()

        # TODO i can't find register telemetry in the swerve module
        # self.drivetrain.register_telemetry( lambda state: self._logger.telemeterize(state) )

    def periodic(self) -> None:
        pose = self.drivetrain.sample_pose_at(Timer.getFPGATimestamp())
        if pose is not None:
            self._latest_pose = pose

        # Dashboard output
        pose_translation = self._latest_pose.translation()
        pose_rotation = self._latest_pose.rotation()
        SmartDashboard.putNumber("Swerve/Pose X (meters)", pose_translation.X())
        SmartDashboard.putNumber("Swerve/Pose Y (meters)", pose_translation.Y())
        SmartDashboard.putNumber("Swerve/Pose Rotation (degrees)", pose_rotation.degrees())


    def get_pose(self) -> Pose2d:
        return self._latest_pose


    ## Methods to control the swerve drive subsystem
    def heading_is_driver_controlled(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(lambda: (
                self._drive_field_centered
                    .with_velocity_x(-self._joystick.getLeftY() * abs(self._joystick.getLeftY()) * self._max_speed)
                    .with_velocity_y(-self._joystick.getLeftX() * abs(self._joystick.getLeftX()) * self._max_speed)
                    .with_rotational_rate(-self._joystick.getRightX() * abs(self._joystick.getRightX() * self._max_angular_rate))
            ))
        )

    def heading_is_auto_controlled(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(lambda: (
                self._drive_facing_direction
                    .with_velocity_x(-self._joystick.getLeftY() * abs(self._joystick.getLeftY()) * self._max_speed)
                    .with_velocity_y(-self._joystick.getLeftX() * abs(self._joystick.getLeftX()) * self._max_speed)
                    .with_target_direction(Rotation2d(0, 1))      # Desired Heading (e.g., (0,1) = 90 deg)
                    .with_heading_pid(1, 0, 0)              # PID for heading control
            ))
        )         

    def pov_move(self, direction_x, direction_y) -> None:
        self.drivetrain.apply_request(
            lambda: self._drive_robot_centered
                .with_velocity_x(direction_x * self._pov_speed)
                .with_velocity_y(direction_y * self._pov_speed)
        )

    def brake(self) -> None:
        self.drivetrain.apply_request(lambda: swerve.requests.SwerveDriveBrake())

    def PIDF_sysID_tuning_bindings(self) -> None:
        (self._joystick.start() & self._joystick.leftBumper()).onTrue(SignalLogger.start)
        (self._joystick.start() & self._joystick.rightBumper()).onTrue(SignalLogger.stop)

        (self._joystick.start() & self._joystick.a()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.b()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._joystick.start() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )


    ## Commands
    def heading_is_auto_controlled_command(self) -> commands2.Command:
        return commands2.cmd.runOnce(self.heading_is_auto_controlled)
    
    def heading_is_driver_controlled_command(self) -> commands2.Command:
        return commands2.cmd.runOnce(self.heading_is_driver_controlled)
    
    def pov_move_command(self, direction_x, direction_y) -> commands2.Command:
        return commands2.cmd.runOnce(lambda: self.pov_move(direction_x, direction_y))

    def brake_command(self) -> commands2.Command:
        return commands2.cmd.runOnce(self.brake)
    
    def reset_field_oriented_perspective(self) -> commands2.Command:
        # Resets the rotation of the robot pose to 0 from the ForwardPerspectiveValue.OPERATOR_PERSPECTIVE perspective. 
        # This makes the current orientation of the robot X forward for field-centric maneuvers.
        return commands2.cmd.runOnce(lambda: self.drivetrain.seed_field_centric())
