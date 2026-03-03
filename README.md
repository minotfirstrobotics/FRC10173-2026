# To Do List:
* Make an intake subsystem (SPARK; running speed like uptake)
* Make a turret subsystem (TalonFX; run to position)
* Camera
  * Restore camera feed
  * Get pose from april tag on dashboard
  * Inject vision pose into swerve drive
* Enable autonomous choose on dashboard
* Build autonomous 
* Determine logging strategies (SignalLogger.enable_auto_logging(True) )

# Dependencies:
* MrK had to use python3.14 -m pip install ... sometimes
* pip install robotpy
* pip install robotpy-commands-v2
* pip install phoenix6
* pip install pynetworktables
* pip install robotpy-pathplannerlib
* pip install robotpy-rev
* pip install photonlibpy

# Python Commands
* robotpy deploy
* py -3 -m robotpy sync (whenever adding/removing module/library)

# CTRE Commands
* ./owlet -f wpilog "input.hoot" "output.wpilog"
