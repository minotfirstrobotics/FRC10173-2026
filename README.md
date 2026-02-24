# Dependencies:
* pip install robotpy
* pip install robotpy-commands-v2
* pip install phoenix6
* pip install pynetworktables
* pip install robotpy-pathplannerlib
* pip install robotpy-rev
* pip install photonlibpy (unused so far)

# Python Commands
* robotpy deploy
* py -3 -m robotpy sync (whenever adding/removing module/library)

# CTRE Commands
* ./owlet -f wpilog "input.hoot" "output.wpilog"

# Enables logging to automatically start at the beginning of an FRC match
# and stop at the end
SignalLogger.enable_auto_logging(True)
