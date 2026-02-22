# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## MINOR

### Added camera_pose.py

- `CameraPose` subsystem for vision-based pose estimation using PhotonVision
- SmartDashboard logging for robot pose from camera vision:
  - `Vision/Pose X` - Robot X position from vision
  - `Vision/Pose Y` - Robot Y position from vision
  - `Vision/Pose Rotation` - Robot rotation angle from vision

### Changed

- Updated camera transform configuration to position camera at center (0, 0) at 0.5m height, facing forward (0 degrees)
- Integrated `CameraPose` subsystem into `robot.py` `RobotContainer` initialization
