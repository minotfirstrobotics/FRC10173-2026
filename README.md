# To Do List:
* get CANDles working
* build extension helper bungies
* Get photonvision apriltagging - app configured and rio coded
* Get photonvision streaming
* calculate range to target
* determine field area and target certain locations
* Determine logging strategies (SignalLogger.enable_auto_logging(True), hoot, etc )

# Dependencies:

* pip install robotpy
* pip install robotpy-commands-v2
* pip install phoenix6
* pip install pynetworktables
* pip install robotpy-pathplannerlib
* pip install robotpy-rev
* pip install photonlibpy
* MrK had to use python3.14 -m pip install ... sometimes

# Python Commands for RoboRIO2
* robotpy deploy
* py -3 -m robotpy sync (whenever adding/removing module/library)

# CTRE Commands
* ./owlet -f wpilog "input.hoot" "output.wpilog"
