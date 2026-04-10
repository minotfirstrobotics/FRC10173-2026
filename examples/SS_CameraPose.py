import commands2
import wpilib
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from wpimath.geometry import Pose3d, Pose2d, Transform3d, Translation3d, Rotation3d
from wpilib import SmartDashboard, Timer
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from photonlibpy import PhotonCamera, PhotonPoseEstimator
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

class SS_CameraPose(commands2.Subsystem):


    def __init__(self, swerve_drive: SS_SwerveDrive):
        super().__init__()
        self.swerve_drive = swerve_drive
        if not wpilib.RobotBase.isReal():
            print("PhotonVision disabled in simulation")
            self.rightcam = None
            self.leftcam = None
            self.estimator = None
            return
        # load load field 
        self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)

        # set camera
        self.rightcam = PhotonCamera("RightCamera") # Left=rightcam, not time to fix
        self.leftcam = PhotonCamera("LeftCamera") # unused for now, but we can add it later if we want

        # we need to measure this 
        self.robot_to_leftcam = Transform3d(
            Translation3d(0.3048, -0.3302, 0.4826), # meters forward, right, up from robot center
            #rotated 180
            Rotation3d(0.0, 0.0, 3.14) # radians roll, pitch, yaw from robot forward
        )
        self.robot_to_rightcam = Transform3d(
            Translation3d(-0.3048, 0.3302, 0.4826), # meters forward, left, up from robot center
            Rotation3d(0.0, 0.0, 3.14) # radians roll, pitch, yaw from robot forward
        )

        # pos estimation
        self.right_estimator = PhotonPoseEstimator(
            self.field_layout,
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
            self.robot_to_rightcam
        )

        self.left_estimator = PhotonPoseEstimator(
            self.field_layout,
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
            self.robot_to_leftcam
        )

    # -----------------------------
    # Prossess Multiple Cameras
    # -----------------------------

    def process_camera_results(self, results, estimator):
        if not results:
            return None

        result = results[-1]
        if not result.hasTargets():
            return None

        # try multitag first
        pose = estimator.estimateCoprocMultiTagPose(result)
        if pose is None:
            pose = estimator.estimateLowestAmbiguityPose(result)
        if pose is None:
            imu_heading = self.swerve_drive.drivetrain.get_state().pose.rotation()
            estimator.addHeadingData(Timer.getFPGATimestamp(), imu_heading)
            pose = estimator.estimatePnpDistanceTrigSolvePose(result)

        return pose

    def combine_poses(self, pose1, pose2):
        if pose1 is None: return pose2
        if pose2 is None: return pose1

        # prefer more tags
        if len(pose1.targetsUsed) > len(pose2.targetsUsed):
            return pose1
        if len(pose2.targetsUsed) > len(pose1.targetsUsed):
            return pose2

        # Prefer lower ambiguity
        if pose1.ambiguity < pose2.ambiguity:
            return pose1
        if pose2.ambiguity < pose1.ambiguity:
            return pose2

        # Prefer newest timestamp
        if pose1.timestampSeconds > pose2.timestampSeconds:
            return pose1
        return pose2

    # -----------------------------
    # Main periodic loop
    # -----------------------------

    def periodic(self):
        right_results = self.rightcam.getAllUnreadResults() if self.rightcam else None
        left_results  = self.leftcam.getAllUnreadResults() if self.leftcam else None

        right_pose = self.process_camera_results(right_results, self.right_estimator)
        left_pose  = self.process_camera_results(left_results, self.left_estimator)

        # Fuse both cameras
        final_pose = self.combine_poses(right_pose, left_pose)
        if final_pose is None:
            return

        pose = final_pose.estimatedPose

        num_tags = len(final_pose.targetsUsed)

        ambiguity = final_pose.ambiguity

        if num_tags >= 2 and ambiguity < 0.1:
            std_devs = [0.1, 0.1, 0.1]   # TRUST ROTATION
        elif num_tags >= 1:
            std_devs = [0.15, 0.15, 0.5] # partial trust
        else:
            return  # shouldn't happen anyway stops it from crashing

        # Inject into drivetrain
        self.swerve_drive.drivetrain.add_vision_measurement(
            pose.toPose2d(),
            final_pose.timestampSeconds,
            vision_measurement_std_devs=std_devs
        )

        # Dashboard
        SmartDashboard.putNumber("Vision/Fused X", pose.X())
        SmartDashboard.putNumber("Vision/Fused Y", pose.Y())
        SmartDashboard.putNumber("Vision/Fused Heading",
                                 pose.rotation().toRotation2d().degrees())