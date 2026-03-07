import commands2
import wpimath.geometry
from wpilib import Timer
from wpilib import SmartDashboard
from photonlibpy import PhotonCamera, PhotonPoseEstimator
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

class SS_CameraPose(commands2.Subsystem):   
    #Subsystem for vision-based pose estimation using PhotonVision

    def __init__(self, swerve_drive) -> None:
        super().__init__()
        self.swerve_drive = swerve_drive

        # Transform from robot to camera
        self.kRobotToCam = wpimath.geometry.Transform3d(
            wpimath.geometry.Translation3d(0.0, 0.0, 0.5),
            wpimath.geometry.Rotation3d.fromDegrees(0.0, 0.0, 0.0),
        )

        # Initialize the camera
        field_layout = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)
        self.cam = PhotonCamera("BackCamera")
        self.cam_pose_est = PhotonPoseEstimator(field_layout, self.kRobotToCam,)
        self.last_pose = None


    def periodic(self) -> None: #Called every 20ms by the CommandScheduler
        results = self.cam.getAllUnreadResults()
        if not results:
            return

        result = results[-1]  # Only use most recent frame

        if not result.hasTargets():
            return

        cam_est_pose = self.cam_pose_est.estimateCoprocMultiTagPose(result)
        if cam_est_pose is None: # fallback 1
            cam_est_pose = self.cam_pose_est.estimateLowestAmbiguityPose(result)
        if cam_est_pose is None: # fallback 2
            imu_heading = self.swerve_drive.drivetrain.get_state().pose.rotation()
            self.cam_pose_est.addHeadingData(Timer.getFPGATimestamp(), imu_heading)
            cam_est_pose = self.cam_pose_est.estimatePnpDistanceTrigSolvePose(result) # needs periodic addHeadingData
        if cam_est_pose is None: # no valid pose could be estimated
            return

        pose3d = cam_est_pose.estimatedPose

        # Reject unrealistic height
        # if abs(pose3d.Z()) > 0.3:
        #     return

        self.last_pose = pose3d

        # Scale trust based on tag count         # Inject into drivetrain pose estimator
        if len(result.targets) >= 2:
            std_devs = (0.3, 0.3, 0.5)
        else:
            std_devs = (0.8, 0.8, 1.5)

        self.swerve_drive.drivetrain.add_vision_measurement(pose3d.toPose2d(), cam_est_pose.timestampSeconds, std_devs)


        # Dashboard output
        pose_translation = pose3d.translation()
        pose_rotation = pose3d.rotation().toRotation2d()

        SmartDashboard.putNumber("Vision/Vision Pose X (m)", pose_translation.X())
        SmartDashboard.putNumber("Vision/Vision Pose Y (m)", pose_translation.Y())
        SmartDashboard.putNumber("Vision/Vision Rotation (deg)", pose_rotation.degrees())
