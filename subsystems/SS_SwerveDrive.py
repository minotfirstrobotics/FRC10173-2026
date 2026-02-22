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
from wpimath.filter import SlewRateLimiter
# Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per second

class SS_SwerveDrive(commands2.SubsystemBase):
    def __init__(self, joystick) -> None:
        super().__init__()
        self._joystick = joystick
        self._max_angular_rate = rotationsToRadians(constants.SWERVE_DEFAULT_NOT_GENERATED["MAX_ROTATION_SPEED"]) # .75 was recommended
        self._max_speed = TunerConstants.speed_at_12_volts * .1
        self._pov_speed = constants.SWERVE_DEFAULT_NOT_GENERATED["MAX_POV_SPEED"]
        self._filter = SlewRateLimiter(0.5)
        self._logger = Telemetry(self._max_speed)

        # Initialize swerve drive configurations
        self._drive_field_centered = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.25)
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
        # self.adv_swerve_bindings()

        # TODO i can't find register telemetry in the swerve module
        # self.drivetrain.register_telemetry(
        #     lambda state: self._logger.telemeterize(state)
        # )
        # TODO this is an example method to add vision to the swerve drive
        # self.drivetrain.add_vision_measurement(vision_robot_pose: Pose2d, timestamp: units.second, vision_measurement_std_devs: tuple[float, float, float] | None = None)


    # def pov_move(self, direction_x, direction_y) -> None:
    #         self.drivetrain.apply_request(
    #             lambda: self._drive_robot_centered.with_velocity_x(
    #                  self._pov_speed * direction_x).with_velocity_y(self._pov_speed * direction_y)
    #         )

    def swerve_bindings(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(lambda: (
                self._drive_field_centered
                    .with_velocity_x(-self._joystick.getLeftY() * self._max_speed)
                    .with_velocity_y(-self._joystick.getLeftX() * self._max_speed)
                    .with_rotational_rate(-self._joystick.getRightX() * self._max_angular_rate)
            ))
        )


    # def swerve_bindings(self) -> None:
    #     self.drivetrain.setDefaultCommand(
    #         self.drivetrain.apply_request(lambda: (
    #             self._drive_field_centered
    #                 .with_velocity_x(self._filter.calculate(-self._joystick.getLeftY() * abs(self._joystick.getLeftY()) * self._max_speed))
    #                 .with_velocity_y(self._filter.calculate(-self._joystick.getLeftX() * abs(self._joystick.getLeftX()) * self._max_speed))
    #                 .with_rotational_rate(self._filter.calculate(-self._joystick.getRightX() * abs(self._joystick.getRightX()) * self._max_angular_rate))
    #         ))
    #     )
        # Resets the rotation of the robot pose to 0 from the ForwardPerspectiveValue.OPERATOR_PERSPECTIVE perspective. 
        # This makes the current orientation of the robot X forward for field-centric maneuvers.
        (self._joystick.back() & self._joystick.start()).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )



    def adv_swerve_bindings(self) -> None:
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )
        (self._joystick.back() & self._joystick.b()).whileTrue( # brake
            self.drivetrain.apply_request(lambda: self._brake)
        )
        (self._joystick.back() & self._joystick.a()).whileTrue( # point wheels at direction
            self.drivetrain.apply_request(
                lambda: self._point_wheels_at_direction.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )
        (self._joystick.start() & self._joystick.a()).whileTrue( # sys_id_dynamic forward
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.b()).whileTrue( # sys_id_dynamic reverse
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._joystick.start() & self._joystick.y()).whileTrue( # sys_id_quasistatic forward
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.x()).whileTrue( # sys_id_quasistatic reverse
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )