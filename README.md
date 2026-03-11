# To Do List:
* Intake is sticking
* Bathtub and belt protection for feeder feeder
* Find CAN errors
* Tune FF for shooter
* Tune PIDF/mm for turret and choose trapezoid vs pidf
* Get 2 cameras working
* Get photonvision for other 2 cams
* Make climbing gear and strategy
* Enable autonomous choice on dashboard
* Build autonomous 
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

# Python Commands
* robotpy deploy
* py -3 -m robotpy sync (whenever adding/removing module/library)

# CTRE Commands
* ./owlet -f wpilog "input.hoot" "output.wpilog"

# Camera USB ports
* USB3 port is back.
* TopRight is front.
* BottomLeft is left.
* BottomRight is right.
