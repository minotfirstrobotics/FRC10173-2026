# PathPlanner 2026 Setup Guide for FRC 10173

## Overview

Your robot is now configured for modern 2026 PathPlanner integration following WPILib best practices. This guide explains what was configured and how to use it.

## What Changed

### 1. **SS_SwerveDrive.py** - AutoBuilder Configuration

Added four critical methods for PathPlanner integration:

- `get_pose()` - Returns current robot position on field
- `reset_pose()` - Resets odometry when an auto starts
- `get_robot_relative_speeds()` - Gets current chassis velocities
- `drive_robot_relative()` - Applies robot-relative velocities from PathPlanner

**Uncommented AutoBuilder Configuration:**

```python
AutoBuilder.configure(
    pose_supplier=self.get_pose,
    reset_pose=self.reset_pose,
    robot_relative_speeds_supplier=self.get_robot_relative_speeds,
    robot_relative_output=self.drive_robot_relative,
    path_following_controller=PPHolonomicDriveController(
        PIDConstants(5.0, 0.0, 0.0),  # Translation PID
        PIDConstants(5.0, 0.0, 0.0),  # Rotation PID
    ),
    robot_config=RobotConfig.fromGUISettings(),
    should_flip_path=lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
    drive_subsystem=self
)
```

### 2. **robot.py** - Auto Execution

- Enabled `AutoBuilder.buildAutoChooser()` to load all `.auto` files from `deploy/pathplanner/autos/`
- Added `getAutonomousCommand()` method in RobotContainer
- Updated `autonomousInit()` to schedule the selected autonomous command

## 2026 Modern Pattern Best Practices

### Named Commands (Already Registered)

Your subsystems already have all essential NamedCommands registered:

**Movement (Automatic via PathPlanner):**

- PathPlanner handles all movement/rotation automatically via `drive_robot_relative()`

**Intake:**

- "Intake Spin" - Run intake forward
- "Intake Reverse" - Run intake backward
- "Intake Stop" - Stop intake

**Shooter:**

- "Shooter Spin-up to Setpoint" - Spin shooter to target RPM
- "Shooter Stop" - Stop shooter

**Feeder:**

- "Feeder Spin" - Run feeder forward
- "Feeder Stop" - Stop feeder
- "Feeder Reverse" - Run feeder backward

**Turret:**

- "Turret Point Ahead" - Point turret forward
- "Turret Point Right" - Point turret right

## Using PathPlanner Autos

### In PathPlanner GUI:

1. Open your `.auto` file (e.g., `simple blue auto.auto`)
2. Create waypoints and paths
3. Add event markers and select the NamedCommands to execute
4. Ensure the final rotation angle is correct for your starting position

### Running Autos:

1. Power on robot (RoboRio 2 on 2026 software)
2. Go to SmartDashboard → "Autonomous Routine" dropdown
3. Select your auto path
4. Enable Autonomou mode - the selected path will run automatically

## Tuning PID Values

The most critical parameters for auto success are the PathPlanner PID gains:

### Translation PID (x/y movement)

Located in `SS_SwerveDrive.py` line ~60:

```python
PIDConstants(5.0, 0.0, 0.0)  # [P, I, D]
```

**Tuning Guide:**

- **P (Proportional)**: Start at 5.0. Increase if robot undershoots paths, decrease if overshooting
- **I (Integral)**: Usually leave at 0.0 for path following
- **D (Derivative)**: Add small amount (0.1-0.5) if oscillating

### Rotation PID (heading correction)

```python
PIDConstants(5.0, 0.0, 0.0)  # [P, I, D]
```

Same tuning principles as translation. Adjust the P value for your robot's angular dynamics.

### Real-Time Tuning via SmartDashboard:

1. While running a test auto, you can adjust gains in PathPlanner
2. The GUI settings are read from `pathplanner/settings.json`
3. Export updated settings back to `tuner_constants_2026_GF.py` if needed

## Command Timing and Sequencing

### Pattern for "Spin up shooter, wait, then shoot":

Use a SequentialCommandGroup in your `.auto` event marker:

In PathPlanner Event Markers, use:

```
Event 1: "Shooter Spin-up to Setpoint"
Event 2: WaitCommand(2.0)  [in code]
Event 3: "Feeder Spin"
```

Or register a composite command in your subsystem:

```python
NamedCommands.registerCommand("Shoot Sequence",
    commands2.SequentialCommandGroup(
        self.spin_up_and_wait_command(),
        feeder.run_forward_command().withTimeout(3.0)
    )
)
```

## 2026 Changes from 2025

### Major Updates:

1. **RoboRio 2**: Faster processing, better odometry support
2. **CTRE Phoenix 6**: Improved swerve drivetrain support with `ApplyRobotSpeeds`
3. **PathPlanner Library**: Updated API for better Commands2 integration
4. **AutoBuilder**: Now properly integrated with modern Command patterns
5. **PID Constants**: Can now be tuned live via PathPlanner GUI settings

### Code Pattern Changes:

- Old: Manual speed conversion with `SwerveRequest` objects
- New: Automatic via `drive_robot_relative()` handling `ChassisSpeeds`

This is more maintainable and follows modern WPILib patterns.

## Troubleshooting

### Auto Not Running:

1. Check SmartDashboard → "Autonomous Routine" shows your auto selected
2. Verify `.auto` files are in `deploy/pathplanner/autos/`
3. Check RioLog for NamedCommand errors
4. Ensure `autonomousInit()` is being called (check robot logs)

### Robot Doesn't Follow Path Correctly:

1. Verify robot pose is updating (SmartDashboard → Swerve Pose X/Y/Rotation)
2. Increase translation P value in AutoBuilder config (start with 7.0)
3. Check that path constraints match your robot's max speeds in `tuner_constants_2026_GF.py`
4. Ensure gyro is zeroed at start (`reset_field_oriented_perspective()`)

### Named Commands Not Executing:

1. Verify NamedCommands are registered in subsystem `__init__`
2. Check exact name matches between PathPlanner event marker and registered command
3. Ensure subsystem requirements are set (`addRequirements()`)

### Slow Movement in Auto:

1. Check `_max_speed_factor` in `SS_SwerveDrive.periodic()` is set properly
2. Verify motor configurations match in `tuner_constants_2026_GF.py`
3. Reduce path velocity constraints in PathPlanner GUI if too aggressive

## Next Steps

1. **Tune Translation/Rotation PID values** via test autos
2. **Create team autos** using PathPlanner GUI with your intake/shooter sequences
3. **Test on practice field** to verify path accuracy (±0.5m is typical)
4. **Optimize max speeds** based on motor specs and game strategy
5. **Add vision corrections** if integrating AprilTag odometry later

## File Structure Reference

```
deploy/pathplanner/
├── settings.json          # Robot config (max speeds, dims)
├── navgrid.json          # Field/obstacle info
├── paths/                # Individual path files
│   ├── FaceRedNorth.path
│   └── SimplePath.path
└── autos/               # Complete auto routines
    └── simple blue auto.auto
```

Each `.auto` file chains paths together and triggers NamedCommands via event markers.

## 2026 Resources

- [CTRE Swerve Docs](https://v6.docs.ctr-electronics.com/en/stable/docs/swerve/swerve-intro.html)
- [PathPlanner Docs](https://pathplannercommunity.github.io/)
- [WPILib Command Framework](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)

---

**Team 10173 - Good luck on the field! 🤖**
