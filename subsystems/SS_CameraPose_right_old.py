import commands2
import wpilib
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from wpimath.geometry import Transform3d, Translation3d, Rotation3d
from wpilib import SmartDashboard, DriverStation
from photonlibpy import PhotonCamera, PhotonPoseEstimator
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

class SS_CameraPose_Right(commands2.Subsystem):
    _MAX_SINGLE_TAG_AMBIGUITY = 0.20
    _MAX_POSE_JUMP_METERS = 0.75

    def __init__(self, swerve_drive: SS_SwerveDrive):
        super().__init__()
        self.swerve_drive = swerve_drive
        if not wpilib.RobotBase.isReal():
            print("PhotonVision disabled in simulation")
            self.rightcam = None
            self.leftcam = None
            self.estimator = None
            return
        # load load field and dashboard toggle
        SmartDashboard.putBoolean("Vision/Enable Right Camera", False)
        SmartDashboard.putBoolean("Vision/Enable In Auto", False)
        SmartDashboard.putNumber("Vision/Max Pose Jump Meters", self._MAX_POSE_JUMP_METERS)
        self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)

        # set camera
        self.rightcam = PhotonCamera("RightCamera") # Left=rightcam, not time to fix
        #self.leftcam = PhotonCamera("LeftCamera") # unused for now, but we can add it later if we want

        # we need to measure this 
        #self.robot_to_leftcam = Transform3d(
            #Translation3d(0.3048, -0.3302, 0.4826), # meters forward, right, up from robot center
            #Rotation3d(0.0, 0.0, 0.0) # radians roll, pitch, yaw from robot forward
        #)
        self.robot_to_rightcam = Transform3d(
            Translation3d(-0.3048, -0.3048, 0.4064), # meters forward, right, up from robot center
            Rotation3d(0.0, 0.0, 3.14) # radians roll, pitch, yaw from robot forward
        )

        # pos estimation
        self.estimator = PhotonPoseEstimator(
            self.field_layout,
            self.robot_to_rightcam,
            #self.robot_to_leftcam
        )

    def periodic(self):
        # grab cams 
        if self.rightcam is None:
            return
        # Dashboard toggle
        if not SmartDashboard.getBoolean("Vision/Enable Right Camera", True):
            return
        if DriverStation.isAutonomousEnabled() and not SmartDashboard.getBoolean("Vision/Enable In Auto", False):
            return
        results = self.rightcam.getAllUnreadResults()
        if not results:
            return

        result = results[-1]
        if not result.hasTargets():
            return

        # Prefer the more stable multitag solve and only fall back when necessary.
        est = self.estimator.estimateCoprocMultiTagPose(result)
        if est is None:
            est = self.estimator.estimateLowestAmbiguityPose(result)

        if est is None:
            return

        target_count = len(est.targetsUsed)
        ambiguity = result.getBestTarget().getPoseAmbiguity()
        max_pose_jump = SmartDashboard.getNumber("Vision/Max Pose Jump Meters", self._MAX_POSE_JUMP_METERS)
        current_pose = self.swerve_drive.get_pose()
        vision_pose = est.estimatedPose.toPose2d()
        pose_jump = current_pose.translation().distance(vision_pose.translation())

        SmartDashboard.putNumber("Vision/Right/Target Count", target_count)
        SmartDashboard.putNumber("Vision/Right/Best Ambiguity", ambiguity)
        SmartDashboard.putNumber("Vision/Right/Pose Jump", pose_jump)

        if target_count < 1:
            return
        if target_count == 1 and ambiguity > self._MAX_SINGLE_TAG_AMBIGUITY:
            return
        if pose_jump > max_pose_jump:
            return

        # send to swerve
        self.swerve_drive.drivetrain.add_vision_measurement(
            vision_pose,
            est.timestampSeconds,
            vision_measurement_std_devs = [.1, .1, 5.0] # distrust of x, y in meters, heading in radians
        )

        # Dashboard
        pose = est.estimatedPose
        SmartDashboard.putNumber("Vision/Vision X right", pose.X())
        SmartDashboard.putNumber("Vision/Vision Y right", pose.Y())
        SmartDashboard.putNumber("Vision/Vision Heading right", 
                                 pose.rotation().toRotation2d().degrees())