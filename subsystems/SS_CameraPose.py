import commands2
import wpilib
import wpimath.geometry
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
        self.cam = PhotonCamera("BackCamera")
        self.cam_pose_est = None
        self.last_pose = None
        self._field_loaded = False


    def add_vision_measurement(self, 
                               vision_pose: wpimath.geometry.Pose2d, 
                               timestamp_seconds: float, 
                               std_devs: tuple[float, float, float] | None = None, 
                               ) -> None:
        self.swerve_drive.drivetrain.add_vision_measurement(vision_pose, timestamp_seconds, std_devs)


    def periodic(self) -> None: #Called every 20ms by the CommandScheduler
        if not self._field_loaded: #Lazy load field layout to avoid blocking initialization
            try:
                # Load the default field layout for AprilTags
                field_layout = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)
                self.cam_pose_est = PhotonPoseEstimator(
                    field_layout,
                    self.kRobotToCam,
                )
                self._field_loaded = True
            except Exception as e:
                wpilib.reportError(f"Failed to load field layout: {e}", printTrace=True)
            return
    
        if self.cam_pose_est is None:
            return
    
        try:
            results = self.cam.getAllUnreadResults()
            if not results:
                return
    
            result = results[-1]  # Only use most recent frame
    
            if not result.hasTargets():
                return
    
            cam_est_pose = self.cam_pose_est.estimateCoprocMultiTagPose(result)

            if cam_est_pose is None: # fallback 1
                cam_est_pose = self.cam_pose_est.estimateLowestAmbiguityPose(result)
    
            # if cam_est_pose is None: # fallback 2
            #     cam_est_pose = self.cam_pose_est.estimatePnpDistanceTrigSolvePose(result) # needs periodic addHeadingData
            #     return

            if cam_est_pose is None: # no valid pose could be estimated
                return
    
            pose3d = cam_est_pose.estimatedPose
    
            # Reject unrealistic height
            if abs(pose3d.Z()) > 0.3:
                return
    
            # Scale trust based on tag count
            if len(result.targets) >= 2:
                std_devs = (0.3, 0.3, 0.5)
            else:
                std_devs = (0.8, 0.8, 1.5)
    
            # Inject into drivetrain pose estimator
            self.add_vision_measurement(pose3d.toPose2d(), cam_est_pose.timestampSeconds, std_devs )

            self.last_pose = pose3d
    
            # Dashboard output
            pose_translation = pose3d.translation()
            pose_rotation = pose3d.rotation().toRotation2d()
    
            SmartDashboard.putNumber("Vision/Pose X (meters)", pose_translation.X())
            SmartDashboard.putNumber("Vision/Pose Y (meters)", pose_translation.Y())
            SmartDashboard.putNumber("Vision/Pose Rotation (degrees)", pose_rotation.degrees())
    
        except Exception as e:
            wpilib.reportError(f"Error in CameraPose periodic: {e}", printTrace=True)
