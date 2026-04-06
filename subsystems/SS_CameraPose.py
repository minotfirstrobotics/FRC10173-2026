import commands2
import wpilib
from wpimath.geometry import Pose3d, Pose2d, Transform3d, Translation3d, Rotation3d
from wpilib import SmartDashboard, Timer
from photonlibpy import PhotonCamera, PhotonPoseEstimator
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

class SS_CameraPose(commands2.Subsystem):

    def __init__(self, swerve_drive):
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
        self.rightcam = PhotonCamera("RightCamera")
        self.leftcam = PhotonCamera("LeftCamera") 

        # we need to measure this 
        self.robot_to_leftcam = Transform3d(
            Translation3d(0.3048, -0.3302, 0.4826), # meters forward, right, up from robot center
            Rotation3d(0.0, 0.0, 0.0) # radians roll, pitch, yaw from robot forward
        )
        self.robot_to_rightcam = Transform3d(
            Translation3d(0.3048, 0.3302, 0.4826), # meters forward, left, up from robot center
            Rotation3d(0.0, 0.0, 0.0) # radians roll, pitch, yaw from robot forward
        )

        # pos estimation
        self.right_estimator = PhotonPoseEstimator(
            self.field_layout,
            self.robot_to_rightcam,
        )
        self.left_estimator = PhotonPoseEstimator(
            self.field_layout,
            self.robot_to_leftcam,
        )

    def _process_camera(self, cam, estimator):
        results = cam.getAllUnreadResults()
        if not results:
            return

        result = results[-1]
        if not result.hasTargets():
            return
        
        # calculate pos based on cams (use one of these)
        #est = self.estimator.estimatePnpDistanceTrigSolvePose(result)
        #est = self.estimator.estimateCoprocMultiTagPose(result)
        est = estimator.estimateLowestAmbiguityPose(result)
        if est is None:
            return

        self.swerve_drive.drivetrain.add_vision_measurement(
            est.estimatedPose.toPose2d(),
            est.timestampSeconds,
            vision_measurement_std_devs=[.3, .3, 5.0]
        )
        return est.estimatedPose
    def periodic(self):
        right_pose = self._process_camera(self.rightcam, self.right_estimator)
        left_pose = self._process_camera(self.leftcam, self.left_estimator)

        if right_pose:
            SmartDashboard.putNumber("Vision/Vision Right Pose X", round(right_pose.X(), 2))
            SmartDashboard.putNumber("Vision/Vision Right Pose Y", round(right_pose.Y(), 2))
            SmartDashboard.putNumber("Vision/Vision Right Pose Heading", round(right_pose.rotation().toRotation2d().degrees(), 1)
            )

        if left_pose:
            SmartDashboard.putNumber("Vision/Vision Left Pose X", round(left_pose.X(), 2))
            SmartDashboard.putNumber("Vision/Vision Left Pose Y", round(left_pose.Y(), 2))
            SmartDashboard.putNumber("Vision/Vision Left Pose Heading", round(left_pose.rotation().toRotation2d().degrees(), 1)
            )