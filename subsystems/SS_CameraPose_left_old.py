import commands2
import wpilib
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from wpimath.geometry import Pose3d, Pose2d, Transform3d, Translation3d, Rotation3d
from wpilib import SmartDashboard, Timer
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from photonlibpy import PhotonCamera, PhotonPoseEstimator
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

class SS_CameraPose_Left(commands2.Subsystem):


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
        SmartDashboard.putBoolean("Vision/Enable Left Camera", True)
        self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)

        # set camera
        self.leftcam = PhotonCamera("LeftCamera") # Left=rightcam, not time to fix
        #self.leftcam = PhotonCamera("LeftCamera") # unused for now, but we can add it later if we want

        # we need to measure this 
        #self.robot_to_leftcam = Transform3d(
            #Translation3d(0.3048, -0.3302, 0.4826), # meters forward, right, up from robot center
            #Rotation3d(0.0, 0.0, 0.0) # radians roll, pitch, yaw from robot forward
        #)
        self.robot_to_leftcam = Transform3d(
            Translation3d(-0.2794, -0.2921, 0.4572), # meters forward, right, up from robot center
            Rotation3d(0.0, 0.0, 3.14) # radians roll, pitch, yaw from robot forward
        )

        # pos estimation
        self.estimator = PhotonPoseEstimator(
            self.field_layout,
            #self.robot_to_rightcam,
            self.robot_to_leftcam
        )

    def periodic(self):
        # grab cams 
        if self.leftcam is None:
            return
        # Dashboard toggle
        if not SmartDashboard.getBoolean("Vision/Enable Left Camera", True):
            return
        results = self.leftcam.getAllUnreadResults()
        if not results:
            return

        result = results[-1]
        if not result.hasTargets():
            return

        # calculate pos based on cams (use one of these)
        #est = self.estimator.estimatePnpDistanceTrigSolvePose(result)
        #est = self.estimator.estimateCoprocMultiTagPose(result)
        est = self.estimator.estimateLowestAmbiguityPose(result)

        if est is None:
            return

        # send to swerve
        self.swerve_drive.drivetrain.add_vision_measurement(
            est.estimatedPose.toPose2d(),
            est.timestampSeconds,
            vision_measurement_std_devs = [.1, .1, 5.0] # distrust of x, y in meters, heading in degrees
        )

        # Dashboard
        pose = est.estimatedPose
        SmartDashboard.putNumber("Vision/Vision X left", pose.X())
        SmartDashboard.putNumber("Vision/Vision Y left", pose.Y())
        SmartDashboard.putNumber("Vision/Vision Heading left",
                                 pose.rotation().toRotation2d().degrees())