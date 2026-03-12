# To Do List:
* try removing redundant command scheduler
* flash radio if necessary
* Intake is sticking
* Bathtub and belt protection for feeder
* Find CAN errors
* Tune FF for shooter
* calculate range to target
* determine field area and target certain locations
* determine how to padlock heading to target point
* Get photonvision for other 2 cams
* Tune PIDF/mm for turret and choose trapezoid vs pidf
* Enable autonomous choice on dashboard
* Build autonomous 
* Make climbing gear and strategy
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

# Camera USB ports on LeftPhotonVision
* USB3 port is back.
* TopRight is front.
* BottomLeft is left.
* BottomRight is right.
