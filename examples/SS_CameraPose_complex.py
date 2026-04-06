import commands2
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d
from wpilib import Timer, SmartDashboard
from photonlibpy import PhotonCamera, PhotonPoseEstimator, EstimatedRobotPose
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

class SS_CameraPose(commands2.Subsystem):   
    #Subsystem for vision-based pose estimation using PhotonVision

    def __init__(self, swerve_drive: SS_SwerveDrive):
        self.swerve_drive = swerve_drive

        # Load the field layout
        field_layout = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)

        # Initialize cameras
        # Define camera-to-robot transforms
        # Initialize pose estimators
        self.front_cam = PhotonCamera("FrontCamera")
        self.kRobotToFrontCam = Transform3d(Translation3d(0.2, 0.0, 0.5), 
                                            Rotation3d(0.0, 0.0, 0.0))  # Example values
        self.front_cam_pose_est = PhotonPoseEstimator(field_layout, self.kRobotToFrontCam)

        # self.back_cam = PhotonCamera("BackCamera")
        # self.kRobotToBackCam = Transform3d(Translation3d(-0.2, 0.0, 0.5), 
        #                                    Rotation3d(0.0, 0.0, 180.0))  # Example values
        # self.back_cam_pose_est = PhotonPoseEstimator(field_layout, self.kRobotToBackCam)

        self.last_pose = None

    def bad_periodic(self) -> None:  # Called every 20ms by the CommandScheduler
        # Get results from both cameras
        # Process the most recent results from each camera
        front_results = self.front_cam.getAllUnreadResults()
        front_pose = self.process_camera_results(front_results, self.front_cam_pose_est)

        # back_results = self.back_cam.getAllUnreadResults()
        # back_pose = self.process_camera_results(back_results, self.back_cam_pose_est)

        # Combine poses: if both are valid
        # if front_pose and back_pose:
        #     self.last_pose = self.combine_poses(front_pose, back_pose)
        if front_pose:
            self.last_pose = front_pose
        # elif back_pose:
        #     self.last_pose = back_pose
        else:
            return  # No valid pose from either camera

        # Inject the pose into the drivetrain pose estimator
        pose3d = self.last_pose.estimatedPose
        # if len(front_results[-1].targets) + len(back_results[-1].targets) >= 2:
        #     std_devs = (0.3, 0.3, 0.5)
        # else:
        #     std_devs = (0.8, 0.8, 1.5)

        self.swerve_drive.drivetrain.add_vision_measurement(
            pose3d.toPose2d(), self.last_pose.timestampSeconds,)# std_devs)

        # Dashboard output
        pose_translation = pose3d.translation()
        pose_rotation = pose3d.rotation().toRotation2d()
        SmartDashboard.putNumber("Vision/Vision Pose X (m)", pose_translation.X())
        SmartDashboard.putNumber("Vision/Vision Pose Y (m)", pose_translation.Y())
        SmartDashboard.putNumber("Vision/Vision Rotation (deg)", pose_rotation.degrees())

    def process_camera_results(self, results, pose_estimator):
        if not results: return None

        result = results[-1]  # Use the most recent frame

        if not result.hasTargets(): return None

        # Try different pose estimation methods
        cam_est_pose = pose_estimator.estimateCoprocMultiTagPose(result)
        if cam_est_pose is None:
            cam_est_pose = pose_estimator.estimateLowestAmbiguityPose(result)
        if cam_est_pose is None:
            imu_heading = self.swerve_drive.drivetrain.get_state().pose.rotation()
            pose_estimator.addHeadingData(Timer.getFPGATimestamp(), imu_heading)
            cam_est_pose = pose_estimator.estimatePnpDistanceTrigSolvePose(result)
        return cam_est_pose

    def combine_poses(self, pose1, pose2):
        # Example: Average the translations and rotations
        avg_translation = (pose1.estimatedPose.translation() + pose2.estimatedPose.translation()) / 2

        rot1 = pose1.estimatedPose.rotation()
        rot2 = pose2.estimatedPose.rotation()
        avg_rotation = Rotation3d(
            (rot1.X() + rot2.X()) / 2,  # Roll
            (rot1.Y() + rot2.Y()) / 2,  # Pitch
            (rot1.Z() + rot2.Z()) / 2   # Yaw
        )

        pose3d = Pose3d(avg_translation, avg_rotation)
        return EstimatedRobotPose(pose3d, pose1.timestampSeconds, pose1.targetsUsed)# + pose2.targetsUsed)
