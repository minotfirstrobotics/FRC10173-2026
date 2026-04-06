import commands2
import wpilib
import math
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpimath.units import rotationsToRadians
from phoenix6 import swerve, SignalLogger
from wpimath.kinematics import ChassisSpeeds
from telemetry import Telemetry
from generated.tuner_constants_2026_GF import TunerConstants
# from generated.tuner_constants_2025_old import TunerConstants
from wpilib import DriverStation, Timer, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d
from commands2.button import Trigger
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.path import PathPlannerPath
from wpilib import DriverStation

class SS_SwerveDrive(commands2.Subsystem):
    def __init__(self, joystick) -> None:
        self.alliance = DriverStation.getAlliance()
        self._joystick = joystick
        self._max_angular_rate = rotationsToRadians(0.75)
        self._max_speed_factor = 0.2
        self._max_speed = self._max_speed_factor * TunerConstants.speed_at_12_volts
        wpilib.SmartDashboard.putNumber("Swerve/Swerve Max Speed Factor", self._max_speed)
        self._pov_speed = 0.2
        self._latest_pose = Pose2d()
        self._last_heading = Rotation2d()
        self.drivetrain = TunerConstants.create_drivetrain() # does this need to after swerve configs?
        # self._logger = Telemetry(self._max_speed)
        # self.drivetrain.register_telemetry( lambda state: self._logger.telemeterize(state) )
        self._left_x_limiter = SlewRateLimiter(2.0)
        self._left_y_limiter = SlewRateLimiter(2.0)
        self._right_x_limiter = SlewRateLimiter(0.25)
        self._right_y_limiter = SlewRateLimiter(1.5)
        self.x_vector_to_target = 0.0
        self.y_vector_to_target = 0.0
        self.range_to_target = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self._padlock_active = False
        self.stop_distance = 1.0 # Default stop distance
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)

        self.PIDF_sysID_tuning_bindings()
        idle = swerve.requests.Idle() # Determine behavior when no other commands are running. 
        Trigger(DriverStation.isDisabled).whileTrue( # This is important to prevent unexpected robot movement when commands end.
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True) )

        # Initialize swerve drive configurations
        self._drive_field_centered = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.1)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE) 
        )
        self._drive_facing_direction = (
            swerve.requests.RobotCentricFacingAngle()
                .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self._drive_field_facing = (
            swerve.requests.FieldCentricFacingAngle()
                .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
                .with_heading_pid(12, 0.0, 5)
        )
        self._drive_robot_centered = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE) 
        )

        self._setup_padlock_target_chooser()
        self._setup_pathplanner_auto_builder()

    def periodic(self) -> None:
        pose = self.drivetrain.sample_pose_at(Timer.getFPGATimestamp())
        if pose is not None:
            self._latest_pose = pose
            self.target_x, self.target_y = self._determine_padlock_target(pose)
            self.x_vector_to_target = self._latest_pose.translation().X() - self.target_x
            self.y_vector_to_target = self._latest_pose.translation().Y() - self.target_y
            self.range_to_target = (self.x_vector_to_target**2 + self.y_vector_to_target**2)**0.5
            self.x_direction_to_target = self.x_vector_to_target / self.range_to_target
            self.y_direction_to_target = self.y_vector_to_target / self.range_to_target
            self.speed_to_target = .8 * (self.range_to_target - self.stop_distance) # Subtract desired stopping distance from error calculation

        dashboard_stop_distance = wpilib.SmartDashboard.getNumber("Swerve/Stop Distance", self.stop_distance)
        dashboard_max_speed = wpilib.SmartDashboard.getNumber("Swerve/Swerve Max Speed Factor", self._max_speed_factor)
        if dashboard_max_speed != self._max_speed_factor:
            self._max_speed_factor = max(min(dashboard_max_speed, 1.0), 0.0) # Clamp between 0 and 1
        if dashboard_stop_distance != self.stop_distance:
            self.stop_distance = max(min(dashboard_stop_distance, 7.0), 0.0)
        wpilib.SmartDashboard.putNumber("Swerve/Target X Vector", self.x_vector_to_target)
        wpilib.SmartDashboard.putNumber("Swerve/Target Y Vector", self.y_vector_to_target)
        wpilib.SmartDashboard.putNumber("Swerve/Target X", self.target_x)
        wpilib.SmartDashboard.putNumber("Swerve/Target Y", self.target_y)
        wpilib.SmartDashboard.putNumber("Swerve/Target Range", self.range_to_target)
        wpilib.SmartDashboard.putNumber("Swerve/Stop Distance", self.stop_distance)

        self._max_speed = self._max_speed_factor * TunerConstants.speed_at_12_volts

        # Dashboard pose output
        pose_translation = self._latest_pose.translation()
        pose_rotation = self._latest_pose.rotation()
        SmartDashboard.putNumber("Swerve/Swerve Pose X (meters)", round(pose_translation.X(), 2))
        SmartDashboard.putNumber("Swerve/Swerve Pose Y (meters)", round(pose_translation.Y(), 2))
        SmartDashboard.putNumber("Swerve/Swerve Rotation (deg)", round(pose_rotation.degrees(), 1))
        self.field.setRobotPose(self._latest_pose)

    def _determine_padlock_target(self, pose: Pose2d) -> tuple:
        selected_target = self._padlock_target_chooser.getSelected()
        if selected_target == (-1.0, -1.0) or selected_target is None: 
            # If "Auto Targetting" is selected, choose target based on alliance and position
            if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
                    selected_target = (4.6, 4.0)

            elif DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                    selected_target = (11.94, 4.0)

        return selected_target

    # -------------------------
    # Drive mode switching for joystick/gamepad control
    # -------------------------
    def drive_mode_field_centered(self) -> None:
            return self.drivetrain.apply_request(lambda: (
                self._drive_field_centered
                    .with_velocity_x(-self._joystick.getLeftY() * abs(self._joystick.getLeftY()) * self._max_speed)
                    .with_velocity_y(-self._joystick.getLeftX() * abs(self._joystick.getLeftX()) * self._max_speed)
                    .with_rotational_rate(-self._joystick.getRightX() * abs(self._joystick.getRightX()) * self._max_angular_rate)
        ))

    def drive_mode_angular(self):
        return self.drivetrain.apply_request(lambda: (
            self._drive_facing_direction
                .with_velocity_x(-self._smoothed_axis(self._joystick.getLeftY(), self._left_y_limiter, square_input=True)* self._max_speed)
                .with_velocity_y(-self._smoothed_axis(self._joystick.getLeftX(), self._left_x_limiter, square_input=True)* self._max_speed)
                .with_target_direction(self._heading_from_right_stick())
                .with_heading_pid(1, 10, 0.2)
        ))
    
    def drive_mode_hybrid(self):
        return self.drivetrain.apply_request(lambda: (
            self._drive_field_facing
                .with_velocity_x(-self._smoothed_axis(self._joystick.getLeftY(), self._left_y_limiter, square_input=True) * self._max_speed)
                .with_velocity_y(-self._smoothed_axis(self._joystick.getLeftX(), self._left_x_limiter, square_input=True) * self._max_speed)
                .with_target_direction(self._heading_from_right_stick())
        ))
    
    def drive_mode_padlocked(self) -> None:
        return self.drivetrain.apply_request(lambda: (
            self._drive_facing_direction
                .with_velocity_x(-self._smoothed_axis(self._joystick.getLeftY(), self._left_y_limiter, square_input=True)* self._max_speed)
                .with_velocity_y(-self._smoothed_axis(self._joystick.getLeftX(), self._left_x_limiter, square_input=True)* self._max_speed)
                .with_target_direction(Rotation2d(self.x_vector_to_target, self.y_vector_to_target))
                .with_heading_pid(12, 0, 5)
        ))

    def drive_mode_robot_centered(self) -> None:
            return self.drivetrain.apply_request(lambda: (
                self._drive_robot_centered
                    .with_velocity_x(-self._joystick.getLeftY() * abs(self._joystick.getLeftY()) * self._max_speed)
                    .with_velocity_y(-self._joystick.getLeftX() * abs(self._joystick.getLeftX()) * self._max_speed)
                    .with_rotational_rate(-self._joystick.getRightX() * abs(self._joystick.getRightX()) * self._max_angular_rate)
        ))

    def _smoothed_axis(self, raw_axis: float, limiter: SlewRateLimiter,
                    deadband: float = 0.12, square_input: bool = False) -> float:
        axis = applyDeadband(raw_axis, deadband)
        if square_input:
            axis = axis * abs(axis)
        return limiter.calculate(axis)


    # -------------------------
    # Drive requests for automated movement
    # -------------------------
    def drive_to_target(self):
        return self.drivetrain.apply_request(lambda: (
            self._drive_field_facing
                .with_velocity_x(self.speed_to_target * self.x_direction_to_target)
                .with_velocity_y(self.speed_to_target * self.y_direction_to_target)
                .with_target_direction(self._heading_from_right_stick())
        ))

    def _heading_from_right_stick(self) -> Rotation2d:
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            ry = -self._joystick.getRightX()
            rx= -self._joystick.getRightY()
        else:
            ry = self._joystick.getRightX()
            rx = self._joystick.getRightY()

        mag = (rx * rx + ry * ry) ** 0.5

        # Only update heading when stick is intentionally moved
        if mag > 0.20:
            self._last_heading = Rotation2d(rx, ry)

        return self._last_heading

    def free_rotate_drive_request_command(self, vx_requested, vy_requested, rotational_rate) -> commands2.Command:
        return self.drivetrain.apply_request(lambda: (
            self._drive_field_centered
                .with_velocity_x(vx_requested * self._max_speed)
                .with_velocity_y(vy_requested * self._max_speed)
                .with_rotational_rate(rotational_rate * self._max_angular_rate) 
        ))

    def padlocked_drive_request_command(self, vx_requested, vy_requested, x_vector=0.0, y_vector=0.0) -> commands2.Command:
        if x_vector == 0 and y_vector == 0:
            x_vector = self.x_vector_to_target
            y_vector = self.y_vector_to_target
        return self.drivetrain.apply_request(lambda: (
            self._drive_facing_direction
                .with_velocity_x(vx_requested * self._max_speed)
                .with_velocity_y(vy_requested * self._max_speed)
                .with_target_direction(Rotation2d(x_vector, y_vector))
                .with_heading_pid(5, 0, 0) 
        ))

    def target_goal(self) -> None:
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            tx, ty = (4.6, 4.0)
        else:
            tx, ty = (12.0, 4.0)

        self._forced_padlock_target = (tx, ty)
        self.target_x = tx
        self.target_y = ty

        try:
            pose = self._latest_pose
            self.x_vector_to_target = tx - pose.translation().X()
            self.y_vector_to_target = ty - pose.translation().Y()
        except:
            self.x_vector_to_target = tx
            self.y_vector_to_target = ty

        self.range_to_target = (self.x_vector_to_target**2 + self.y_vector_to_target**2)**0.5
        wpilib.SmartDashboard.putBoolean("Swerve/Padlock Engaged", True)

    def release_padlock_goal(self):
        self._padlock_active = False
        wpilib.SmartDashboard.putBoolean("Swerve/Padlock Engaged", False)

    def hold_padlock_goal_command(self) -> commands2.Command:
        return commands2.cmd.startEnd(
            self.target_goal,
            self.release_padlock_goal,
            self,
        )

    def robot_pov_drive_request_command(self, direction_x, direction_y) -> commands2.Command:
        return self.drivetrain.apply_request(lambda: (
            self._drive_robot_centered
                .with_velocity_x(direction_x * self._pov_speed)
                .with_velocity_y(direction_y * self._pov_speed)
                .with_rotational_rate(0) 
        ))

    def brake(self) -> None:
        self.drivetrain.apply_request(lambda: swerve.requests.SwerveDriveBrake())

    # -------------------------
    # Utility functions
    # -------------------------
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

    def reset_field_oriented_perspective(self) -> None:
        # Resets the rotation of the robot pose to 0 from the ForwardPerspectiveValue.OPERATOR_PERSPECTIVE perspective. 
        # This makes the current orientation of the robot X forward for field-centric maneuvers.
        return self.drivetrain.seed_field_centric()


    # -------------------------
    # Pathplannerlib setup and helpers
    # -------------------------
    def _setup_pathplanner_auto_builder(self) -> None:
        AutoBuilder.configure(
            pose_supplier=self.get_pose,
            reset_pose=self.reset_pose,
            robot_relative_speeds_supplier=self.get_robot_relative_speeds,
            output=self.drive_robot_relative,
            controller=PPHolonomicDriveController(
                PIDConstants(2.0, 0.0, 0.0),  # Translation PID (tune these values)
                PIDConstants(2.0, 0.0, 0.0),  # Rotation PID (tune these values)
            ),
            robot_config=RobotConfig.fromGUISettings(),
            should_flip_path=lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
            drive_subsystem=self
        )

        path_1m = PathPlannerPath.fromPathFile("1m fowd")  # name matches the .path file
        follow_command_1m = AutoBuilder.followPath(path_1m)
        SmartDashboard.putData("Commands/Swerve/Follow 1m Fwd Path", follow_command_1m)

        path_jerk = PathPlannerPath.fromPathFile("Jerk(F-B)")  # name matches the .path file
        follow_command_jerk = AutoBuilder.followPath(path_jerk)
        SmartDashboard.putData("Commands/Swerve/Jerk(F-B)", follow_command_jerk)

        path_trench = PathPlannerPath.fromPathFile("Under Trench Test")  # name matches the .path file
        follow_command_trench = AutoBuilder.followPath(path_trench)
        SmartDashboard.putData("Commands/Swerve/Trench", follow_command_trench)


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

    def _setup_padlock_target_chooser(self):
        # This is an example of how you might set up a dashboard chooser to select between different padlock targets (e.g., different scoring locations)
        self._padlock_target_chooser = wpilib.SendableChooser()
        self._padlock_target_chooser.setDefaultOption("Auto Targetting", (-1.0, -1.0)) # Choose a target based on alliance and position
        self._padlock_target_chooser.addOption("Blue Target", (4.6, 4.0)) # Blue alliance target
        self._padlock_target_chooser.addOption("Blue Top Zone", (4.0, 6.0)) # Red alliance target
        self._padlock_target_chooser.addOption("Blue Bottom Zone", (4.0, 2.0)) # Red alliance target
        self._padlock_target_chooser.addOption("Red Target", (12.0, 4.0)) # Red alliance target
        self._padlock_target_chooser.addOption("Red Top Zone", (12.6, 6.0)) # Red alliance target
        self._padlock_target_chooser.addOption("Red Bottom Zone", (12.6, 2.0)) # Red alliance target
        wpilib.SmartDashboard.putData("Swerve/Padlock Target Chooser", self._padlock_target_chooser)