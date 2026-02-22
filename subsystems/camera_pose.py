import commands2
import wpilib
import wpimath.geometry
from wpilib import SmartDashboard
from photonlibpy import PhotonCamera, PhotonPoseEstimator
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout


class CameraPose(commands2.SubsystemBase):
    """Subsystem for vision-based pose estimation using PhotonVision"""

    def __init__(self, swerve_drive) -> None:
        """
        Initialize the Camera Pose subsystem.
        
        Args:
            swerve_drive: Reference to the swerve drive subsystem for adding vision measurements
        """
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

    def _load_field_layout(self) -> None:
        """Lazy load field layout to avoid blocking initialization"""
        if not self._field_loaded:
            try:
                # Load the default field layout for AprilTags
                field_layout = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)
                self.cam_pose_est = PhotonPoseEstimator(
                    field_layout,
                    PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                    self.cam,
                    self.kRobotToCam,
                )
                self._field_loaded = True
            except Exception as e:
                wpilib.reportError(f"Failed to load field layout: {e}", printTraceback=True)

    def periodic(self) -> None:
        """Called every 20ms by the CommandScheduler"""
        # Lazy load field layout on first periodic call
        if not self._field_loaded:
            self._load_field_layout()
            return

        # If the pose estimator is not initialized, skip processing
        if self.cam_pose_est is None:
            return

        try:
            # Process all unread results from the camera
            for result in self.cam.getAllUnreadResults():
                cam_est_pose = self.cam_pose_est.estimateCoprocMultiTagPose(result)

                if cam_est_pose is None:
                    cam_est_pose = self.cam_pose_est.estimateLowestAmbiguityPose(result)

                if cam_est_pose is not None:
                    # Add vision measurement to swerve drive
                    if hasattr(self.swerve_drive.drivetrain, 'add_vision_measurement'):
                        self.swerve_drive.drivetrain.add_vision_measurement(
                            cam_est_pose.estimatedPose, cam_est_pose.timestampSeconds
                        )
                    self.last_pose = cam_est_pose.estimatedPose

            # Send pose data to SmartDashboard
            if self.last_pose is not None:
                pose_translation = self.last_pose.translation()
                pose_rotation = self.last_pose.rotation()

                SmartDashboard.putNumber("Vision/Pose X (meters)", pose_translation.X())
                SmartDashboard.putNumber("Vision/Pose Y (meters)", pose_translation.Y())
                SmartDashboard.putNumber("Vision/Pose Rotation (degrees)", pose_rotation.degrees())
        except Exception as e:
            wpilib.reportError(f"Error in CameraPose periodic: {e}", printTraceback=True)