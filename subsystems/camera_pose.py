###################################################################################
# MIT License
#
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

        self.kRobotToCam = wpimath.geometry.Transform3d(
            wpimath.geometry.Translation3d(0.0, 0.0, 0.5),
            wpimath.geometry.Rotation3d.fromDegrees(0.0, 0.0, 0.0),
        )

        self.cam = PhotonCamera("FrontCamera")
        self.cam_pose_est = None
        self.last_pose = None
        self._field_loaded = False

    def _load_field_layout(self) -> None:
        """Lazy load field layout to avoid blocking initialization"""
        if not self._field_loaded:
            try:
                self.cam_pose_est = PhotonPoseEstimator(
                    AprilTagFieldLayout.loadField(AprilTagField.kDefaultField),
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
        
        if self.cam_pose_est is None:
            return

        try:
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
                
                SmartDashboard.putNumber("Vision/Pose X", pose_translation.X())
                SmartDashboard.putNumber("Vision/Pose Y", pose_translation.Y())
                SmartDashboard.putNumber("Vision/Pose Rotation", pose_rotation.degrees())
        except Exception as e:
            wpilib.reportError(f"Error in CameraPose periodic: {e}", printTraceback=True)


