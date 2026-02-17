#####################################
from pathplannerlib.auto import NamedCommands

class RobotContainer:
    def __init__(self):
        # Initialize subsystems (e.g., self.swerve = SwerveSubsystem())
        
        # Register named commands before building autos
        NamedCommands.registerCommand('intakeDown', self.intake.intakeDownCommand())
        NamedCommands.registerCommand('shootNote', self.shooter.shootCommand())


###################################
from pathplannerlib.auto import PathPlannerAuto

def getAutonomousCommand(self):
    return PathPlannerAuto('Example Auto')


###################################
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

def getPathCommand(self):
    path = PathPlannerPath.fromPathFile('TestPath')
    return AutoBuilder.followPath(path)
