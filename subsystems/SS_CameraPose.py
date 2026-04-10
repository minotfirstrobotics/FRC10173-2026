import commands2
import wpilib
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from wpimath.geometry import Transform3d, Translation3d, Rotation3d
from wpilib import SmartDashboard
from photonlibpy import PhotonCamera, PhotonPoseEstimator
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

class SS_CameraPose(commands2.Subsystem):


    def __init__(self, swerve_drive: SS_SwerveDrive, photon_vision_name: str="", 
                 cam_location_fwd: float=0.0, cam_location_left: float=0.0, cam_location_up: float=0.0, 
                 cam_rotation_yaw: float=0.0, cam_rotation_pitch: float=0.0):
        self.swerve_drive = swerve_drive
        self.photon_vision_name = photon_vision_name
        if not wpilib.RobotBase.isReal():
            print("PhotonVision disabled in simulation")
            self.cam = None
            self.estimator = None
            return

        SmartDashboard.putBoolean(f"Vision/Enable {self.photon_vision_name} Camera", True)
        self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)

        self.cam = PhotonCamera(f"{self.photon_vision_name}Camera")

        self.robot_to_cam = Transform3d(
            Translation3d(cam_location_fwd, cam_location_left, cam_location_up), # meters forward, left, up from robot center
            Rotation3d(0.0, cam_rotation_pitch, cam_rotation_yaw) # radians roll, pitch, yaw from robot forward
        )

        self.estimator = PhotonPoseEstimator(
            self.field_layout,
            self.robot_to_cam
        )

    def periodic(self):
        if self.cam is None:
            return
        if not SmartDashboard.getBoolean(f"Vision/Enable {self.photon_vision_name} Camera", True):
            return
        results = self.cam.getAllUnreadResults()
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

        self.swerve_drive.drivetrain.add_vision_measurement(
            est.estimatedPose.toPose2d(),
            est.timestampSeconds,
            vision_measurement_std_devs = [.1, .1, 5.0] # distrust of x/y in m, heading in radians
        )

        pose = est.estimatedPose
        SmartDashboard.putNumber(f"Vision/Vision X {self.photon_vision_name}", pose.X())
        SmartDashboard.putNumber(f"Vision/Vision Y {self.photon_vision_name}", pose.Y())
        SmartDashboard.putNumber(f"Vision/Vision Heading {self.photon_vision_name}",
                                 pose.rotation().toRotation2d().degrees())