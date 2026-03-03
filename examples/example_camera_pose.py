"""
## Adding Explicit Camera Source to Dashboard Example
# ...existing code...
from cscore import CameraServer, HttpCamera, VideoSource
from wpilib import SmartDashboard
# ...existing code...

class SS_CameraPose(commands2.Subsystem):
    def __init__(self, drivetrain):
        # ...existing code...
        self.drivetrain = drivetrain
        self.camera = PhotonCamera("BackCamera")
        # ...existing code...

        # Explicit dashboard camera source (PhotonVision MJPEG stream)
        stream_url = "http://photonvision.local:1182/?action=stream"
        self.dashboard_cam = HttpCamera(
            "Vision_BackCamera",
            stream_url,
            HttpCamera.HttpCameraKind.kMJPGStreamer
        )
        self.dashboard_cam.setConnectionStrategy(
            VideoSource.ConnectionStrategy.kKeepOpen
        )
        CameraServer.addCamera(self.dashboard_cam)

        # SmartDashboard/vision category metadata
        SmartDashboard.putString("Vision/CameraSource", "Vision_BackCamera")
        SmartDashboard.putString("Vision/CameraURL", stream_url)
        SmartDashboard.putBoolean("Vision/CameraPublished", True)

        # ...existing code...



import drivetrain
import wpilib
import wpimath.geometry
from photonlibpy import PhotonCamera, PhotonPoseEstimator
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

kRobotToCam = wpimath.geometry.Transform3d(
    wpimath.geometry.Translation3d(0.5, 0.0, 0.5),
    wpimath.geometry.Rotation3d.fromDegrees(0.0, -30.0, 0.0),
)


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        #Robot initialization function
        self.controller = wpilib.XboxController(0)
        self.swerve = drivetrain.Drivetrain()
        self.cam = PhotonCamera("YOUR CAMERA NAME")
        self.camPoseEst = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.kDefaultField),
            kRobotToCam,
        )

    def robotPeriodic(self) -> None:
        for result in self.cam.getAllUnreadResults():
            camEstPose = self.camPoseEst.estimateCoprocMultiTagPose(result)
            if camEstPose is None:
                camEstPose = self.camPoseEst.estimateLowestAmbiguityPose(result)

            self.swerve.addVisionPoseEstimate(
                camEstPose.estimatedPose, camEstPose.timestampSeconds
            )

        self.swerve.updateOdometry()
        self.swerve.log()

    def teleopPeriodic(self) -> None:
        xSpeed = -1.0 * self.controller.getLeftY() * drivetrain.kMaxSpeed
        ySpeed = -1.0 * self.controller.getLeftX() * drivetrain.kMaxSpeed
        rot = -1.0 * self.controller.getRightX() * drivetrain.kMaxAngularSpeed

        self.swerve.drive(xSpeed, ySpeed, rot, True, self.getPeriod())

    def _simulationPeriodic(self) -> None:
        self.swerve.simulationPeriodic()
        return super()._simulationPeriodic()

#!/usr/bin/env python3
###################################################################################
# MIT License
#
# Copyright (c) PhotonVision
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
###################################################################################


"""