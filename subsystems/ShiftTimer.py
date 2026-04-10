import wpilib
import commands2

class ShiftTimer():
    def __init__(self, match_timer: wpilib.Timer) -> None:
        self.match_timer = match_timer
        self.shooting_shift = False
        self.next_shift_end = 10
        self.shift_schedule = [ ("Transition", 140),
                                ("Shift 1", 130),
                                ("Shift 2", 105),
                                ("Shift 3", 80),
                                ("Shift 4", 55),
                                ("End Game", 30),
                                ]
        self.current_shift = "Transition"
        self.update_dashboard()

    def update_dashboard(self):
        self.shooting_shift = not self.shooting_shift
        wpilib.SmartDashboard.putString("Current Shift", self.current_shift)
        wpilib.SmartDashboard.putBoolean("Shooting Shift Active", self.shooting_shift)

    def its_periodic(self):
        on_shift = wpilib.SmartDashboard.getBoolean("Shooting Shift Active", self.shooting_shift)
        if on_shift != self.shooting_shift:
            self.shooting_shift = on_shift
        current_time = self.match_timer.get()

        remaining_shift_time = current_time - self.next_shift_end
        wpilib.SmartDashboard.putNumber("Time Remaining", self.next_shift_end - self.match_timer.get())

        match self.current_shift:
            case "Transition":
                if current_time >= 10:
                    self.current_shift = "Shift 1"
                    self.next_shift_end = 35
                    self.update_dashboard()
            case "Shift 1":
                if current_time >= self.next_shift_end:
                    self.current_shift = "Shift 2"
                    self.next_shift_end = 60
                    self.update_dashboard()
            case "Shift 2":
                if current_time >= self.next_shift_end:
                    self.current_shift = "Shift 3"
                    self.next_shift_end = 85
                    self.update_dashboard()
            case "Shift 3":
                if current_time >= self.next_shift_end:
                    self.current_shift = "Shift 4"
                    self.next_shift_end = 110
                    self.update_dashboard()
            case "Shift 4":
                if current_time >= self.next_shift_end:
                    self.current_shift = "End Game"
                    self.shooting_shift = False # to force endgame to invert to true
                    self.next_shift_end = 140
                    self.update_dashboard()
