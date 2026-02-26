# ## Example command definition
# import commands2
# from subsystems.shooter import ShooterSubsystem
# import wpilib

# class Shoot(commands2.Command):
#     def __init__(self, shooter: ShooterSubsystem):
#         super().__init__()
#         self.shooter = shooter
#         self.addRequirements(shooter) # Ensure no other command uses shooter
#         self.timer = wpilib.Timer()

#     def initialize(self):
#         self.timer.restart()
#         self.shooter.setSpeed(0.5) # Spin up

#     def execute(self):
#         pass # Command is running

#     def isFinished(self):
#         return self.timer.hasElapsed(2.0) # Finish after 2 seconds

#     def end(self, interrupted):
#         self.shooter.setSpeed(0) # Stop
        


# ## Example command sequence definition
# # autonomous_sequence.py
# import commands2
# from commands2 import SequentialCommandGroup, WaitCommand
# # Assuming these are defined in other files
# from commands.drive_forward import DriveForward
# from commands.turn import Turn

# class AutonomousSequence(SequentialCommandGroup):
#     """
#     A sequence of commands: drive forward, wait, then turn.
#     """
#     def __init__(self, drive_subsystem, intake_subsystem):
#         super().__init__(
#             DriveForward(drive_subsystem, distance=10).withTimeout(5),
#             WaitCommand(1.0),
#             Turn(drive_subsystem, angle=90),
#             # something here with intake_subsystem
#         )



# ## Usage in main code
# from autonomous_sequence import AutonomousSequence

# # In robotInit or autonomousInit
# self.auto = AutonomousSequence(self.drive, self.intake)
# self.auto.schedule()
