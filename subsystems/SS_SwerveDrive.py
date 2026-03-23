import commands2
import wpilib
from wpimath.units import rotationsToRadians
from phoenix6 import swerve, SignalLogger
from wpimath.kinematics import ChassisSpeeds
from telemetry import Telemetry
# from generated.tuner_constants_2026_GF import TunerConstants
from generated.tuner_constants_2025_old import TunerConstants
from wpilib import DriverStation, Timer, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d
from commands2.button import Trigger
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController
from wpilib import DriverStation


class SS_SwerveDrive(commands2.Subsystem):
    def __init__(self, joystick) -> None:
        super().__init__()
        self._joystick = joystick
        self._max_angular_rate = rotationsToRadians(.75)
        self._max_speed_factor = 0.2
        self._max_speed = self._max_speed_factor * TunerConstants.speed_at_12_volts
        wpilib.SmartDashboard.putNumber("SS_Telemetry/Swerve Max Speed", self._max_speed)
        self._pov_speed = 0.2
        self._latest_pose = Pose2d()
        self.drivetrain = TunerConstants.create_drivetrain() # does this need to after swerve configs?
        # self._logger = Telemetry(self._max_speed)
        # self.drivetrain.register_telemetry( lambda state: self._logger.telemeterize(state) )

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)

        self.PIDF_sysID_tuning_bindings()

        idle = swerve.requests.Idle() # Determine behavior when no other commands are running. 
        Trigger(DriverStation.isDisabled).whileTrue( # This is important to prevent unexpected robot movement when commands end.
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        # Initialize swerve drive configurations
        self._drive_field_centered = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.1)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self._drive_facing_direction = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.1)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self._drive_robot_centered = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self.heading_is_driver_controlled()

        AutoBuilder.configure(
            pose_supplier=self.get_pose,
            reset_pose=self.reset_pose,
            robot_relative_speeds_supplier=self.get_robot_relative_speeds,
            output=self.drive_robot_relative,
            controller=PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),  # Translation PID (tune these values)
                PIDConstants(5.0, 0.0, 0.0),  # Rotation PID (tune these values)
            ),
            robot_config=RobotConfig.fromGUISettings(),
            should_flip_path=lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
            drive_subsystem=self
        )
    
    def periodic(self) -> None:
        pose = self.drivetrain.sample_pose_at(Timer.getFPGATimestamp())
        if pose is not None:
            self._latest_pose = pose

        if wpilib.SmartDashboard.getNumber("SS_Telemetry/Swerve Max Speed", self._max_speed_factor) != self._max_speed_factor:
            self._max_speed_factor = wpilib.SmartDashboard.getNumber("SS_Telemetry/Swerve Max Speed", self._max_speed_factor)
            # place to update any commands that rely on _max_speed if needed
        if self._max_speed_factor > 1: # Cap at Drive Speed
            self._max_speed_factor = 1
            wpilib.SmartDashboard.putNumber("SS_Telemetry/Swerve Max Speed", self._max_speed_factor)
        
        self._max_speed = self._max_speed_factor * TunerConstants.speed_at_12_volts

        # Dashboard output
        pose_translation = self._latest_pose.translation()
        pose_rotation = self._latest_pose.rotation()
        SmartDashboard.putNumber("Swerve/Swerve Pose X (meters)", pose_translation.X())
        SmartDashboard.putNumber("Swerve/Swerve Pose Y (meters)", pose_translation.Y())
        SmartDashboard.putNumber("Swerve/Swerve Rotation (deg)", pose_rotation.degrees())
        self.field.setRobotPose(self._latest_pose)

    # -------------------------
    # is the pose for pathplannerlib
    # -------------------------
    def get_pose(self) -> Pose2d:
        return self._latest_pose

    def reset_pose(self, pose: Pose2d) -> None:
        self.drivetrain.reset_pose(pose)
        self._latest_pose = pose

    def get_robot_relative_speeds(self) -> ChassisSpeeds:
        """Get the current robot-relative chassis speeds.
        The underlying drivetrain may not yet have a valid state (especially
        during early init or in simulation). Return a zero ChassisSpeeds if the
        state is unavailable to avoid AttributeError.
        """
        state = None
        try:
            state = self.drivetrain.get_state()
        except Exception:
            # Be defensive: if getting state raises, treat as no motion.
            state = None

        if state is None:
            return ChassisSpeeds(0.0, 0.0, 0.0)

        # Some drivetrain implementations may not include 'speeds' attribute
        # (unlikely), so be defensive.
        return getattr(state, "speeds", ChassisSpeeds(0.0, 0.0, 0.0))

    def drive_robot_relative(self, robot_relative_speeds: ChassisSpeeds, drive_feedforwards=None) -> None:
        self.drivetrain.set_control(
            swerve.requests.ApplyRobotSpeeds().with_speeds(robot_relative_speeds)
        )

    # -------------------------
    # Motor movement functions
    # -------------------------
    def heading_is_driver_controlled(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(lambda: (
                self._drive_field_centered
                    .with_velocity_x(-self._joystick.getLeftY() * abs(self._joystick.getLeftY()) * self._max_speed)
                    .with_velocity_y(-self._joystick.getLeftX() * abs(self._joystick.getLeftX()) * self._max_speed)
                    .with_rotational_rate(-self._joystick.getRightX() * abs(self._joystick.getRightX() * self._max_angular_rate)) )))

    def heading_is_auto_controlled(self, vx_requested, vy_requested, vrotation_requested) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(lambda: (
                self._drive_field_centered
                    .with_velocity_x(vx_requested * self._max_speed)
                    .with_velocity_y(vy_requested * self._max_speed)
                    .with_rotational_rate(vrotation_requested * self._max_angular_rate)) ))

    def heading_is_driver_padlocked(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(lambda: (
                self._drive_facing_direction
                    .with_velocity_x(-self._joystick.getLeftY() * abs(self._joystick.getLeftY()) * self._max_speed)
                    .with_velocity_y(-self._joystick.getLeftX() * abs(self._joystick.getLeftX()) * self._max_speed)
                    .with_target_direction(Rotation2d(0, 1))      # Desired Heading (e.g., (0,1) = 90 deg)
                    .with_heading_pid(1, 0, 0) ))  )             # PID for heading control

    def heading_is_auto_padlocked(self, vx_requested, vy_requested, x_vector, y_vector) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(lambda: (
                self._drive_facing_direction
                    .with_velocity_x(vx_requested * self._max_speed)
                    .with_velocity_y(vy_requested * self._max_speed)
                    .with_target_direction(Rotation2d(x_vector, y_vector))      # Desired Heading (e.g., (0,1) = 90 deg)
                    .with_heading_pid(1, 0, 0) ))  )             # PID for heading control
            
    def pov_move(self, direction_x, direction_y) -> None:
        self.drivetrain.apply_request(lambda: (
            self._drive_robot_centered
                .with_velocity_x(direction_x * self._pov_speed)
                .with_velocity_y(direction_y * self._pov_speed)
            )
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


    # -------------------------
    # Commands
    # -------------------------
    def heading_is_driver_controlled_command(self) -> commands2.Command:
        return commands2.cmd.runOnce(self.heading_is_driver_controlled)
    
    def heading_is_auto_controlled_command(self, vx_requested=0.0, vy_requested=0.0, vrotation_requested=0.0) -> commands2.Command:
        return commands2.cmd.runOnce(lambda: self.heading_is_auto_controlled(vx_requested, vy_requested, vrotation_requested))

    def heading_is_driver_padlocked_command(self) -> commands2.Command:
        return commands2.cmd.runOnce(self.heading_is_driver_padlocked)
    
    def heading_is_auto_padlocked_command(self, vx_requested=0.0, vy_requested=0.0, x_vector=0.0, y_vector=0.2) -> commands2.Command:
        return commands2.cmd.runOnce(lambda: self.heading_is_auto_padlocked(vx_requested, vy_requested, x_vector, y_vector))
    
    def pov_move_command(self, direction_x=0.0, direction_y=0.0) -> commands2.Command:
        return commands2.cmd.runOnce(lambda: self.pov_move(direction_x, direction_y))

    def brake_command(self) -> commands2.Command:
        return commands2.cmd.runOnce(self.brake)
    
    def reset_field_oriented_perspective_command(self) -> commands2.Command:
        # Resets the rotation of the robot pose to 0 from the ForwardPerspectiveValue.OPERATOR_PERSPECTIVE perspective. 
        # This makes the current orientation of the robot X forward for field-centric maneuvers.
        return commands2.cmd.runOnce(lambda: self.drivetrain.seed_field_centric())
