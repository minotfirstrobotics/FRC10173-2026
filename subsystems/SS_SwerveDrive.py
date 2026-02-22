import commands2
import constants
from wpilib import DriverStation
from wpimath.geometry import Rotation2d
from wpimath.filter import SlewRateLimiter
from wpimath.units import rotationsToRadians
from phoenix6 import swerve
from telemetry import Telemetry
from generated.tuner_constants import TunerConstants
from commands2.button import Trigger
from commands2.sysid import SysIdRoutine

class SS_SwerveDrive(commands2.SubsystemBase):
    def __init__(self, joystick) -> None:
        super().__init__()
        self._joystick = joystick

        self._max_angular_rate = rotationsToRadians(constants.SWERVE_DEFAULT_NOT_GENERATED["MAX_ROTATION_SPEED"])
        self._max_speed = TunerConstants.speed_at_12_volts * 0.25
        self._pov_speed = constants.SWERVE_DEFAULT_NOT_GENERATED["MAX_POV_SPEED"]

        self._linear_filter = SlewRateLimiter(0.5)  # X/Y linear
        self._angular_filter = SlewRateLimiter(1.0)  # rotation

        self._logger = Telemetry(self._max_speed)

        self._drive_field_centered = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.1)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self._drive_robot_centered = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point_wheels_at_direction = swerve.requests.PointWheelsAt()

        self.drivetrain = TunerConstants.create_drivetrain()

        self.swerve_bindings()

    def swerve_bindings(self) -> None:
        def drive_request():
            raw_x = -self._joystick.getLeftY()
            raw_y = -self._joystick.getLeftX()
            raw_rot = -self._joystick.getRightX()

            deadband = 0.05
            raw_x = 0.0 if abs(raw_x) < deadband else raw_x
            raw_y = 0.0 if abs(raw_y) < deadband else raw_y
            raw_rot = 0.0 if abs(raw_rot) < deadband else raw_rot

            x = self._linear_filter.calculate(raw_x * self._max_speed)
            y = self._linear_filter.calculate(raw_y * self._max_speed)
            rot = self._angular_filter.calculate(raw_rot * self._max_angular_rate)

            return self._drive_field_centered.with_velocity_x(x).with_velocity_y(y).with_rotational_rate(rot)

        self.drivetrain.setDefaultCommand(self.drivetrain.apply_request(drive_request))

        (self._joystick.back() & self._joystick.start()).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

    def adv_swerve_bindings(self) -> None:
        idle = swerve.requests.Idle()

        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        (self._joystick.back() & self._joystick.b()).whileTrue(
            self.drivetrain.apply_request(lambda: self._brake)
        )

        (self._joystick.back() & self._joystick.a()).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point_wheels_at_direction.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )

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