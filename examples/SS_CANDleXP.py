import wpilib
import commands2
from phoenix6.hardware import CANdle
from phoenix6.configs import CANdleConfiguration
from phoenix6.configs.config_groups import CANdleFeaturesConfigs
from phoenix6.signals import StripTypeValue, RGBWColor
from phoenix6.controls import FireAnimation, SingleFadeAnimation, TwinkleAnimation, LarsonAnimation  
from phoenix6.controls import TwinkleOffAnimation, SolidColor, ColorFlowAnimation

class SS_CANdleLight(commands2.Subsystem):
    def __init__(self, CANdle_channel: int, canbus, dashboard_name: str) -> None:
        self.candle = CANdle(CANdle_channel, canbus)
        self.dashboard_name = dashboard_name
        configs = CANdleConfiguration()
        configs.led.with_strip_type(StripTypeValue.RGB).with_brightness_scalar(0.5)

        configs.candle_features = CANdleFeaturesConfigs()
        #configs.candle_features.with_enable5_v_rail(True)
        self.candle.configurator.apply(configs)
        self.rgb_white = 0
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            self.rgb_red = 255
            self.rgb_green = 0
            self.rgb_blue = 0
            alliance_color_solid = lambda: self.set_all_leds_RGBW(red=self.rgb_red, green=self.rgb_green, blue=self.rgb_blue)
        elif wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue:
            self.rgb_red = 0
            self.rgb_green = 0
            self.rgb_blue = 255
            alliance_color_solid = lambda: self.set_all_leds_RGBW(red=self.rgb_red, green=self.rgb_green, blue=self.rgb_blue)
        else:
            self.rgb_red = 255
            self.rgb_green = 255
            self.rgb_blue = 255
            alliance_color_solid = lambda: self.set_all_leds_RGBW(red=self.rgb_red, green=self.rgb_green, blue=self.rgb_blue)
        self._color_chooser = wpilib.SendableChooser()
        self._color_chooser.setDefaultOption("Alliance Color Solid", alliance_color_solid) # Choose a target based on alliance and position
        self._color_chooser.addOption("Fire", self.set_animation_fire)
        self._color_chooser.addOption("Twinkle", self.set_animation_twinkle)
        self._color_chooser.addOption("Single Fade", lambda: self.set_animation_single_fade(red=self.rgb_red, green=self.rgb_green, blue=self.rgb_blue))
        self._color_chooser.addOption("Larson", lambda: self.set_animation_larson(red=self.rgb_red, green=self.rgb_green, blue=self.rgb_blue))
        self._color_chooser.addOption("Color Flow", self.set_animation_color_flow)
        self._color_chooser.addOption("Twinkle Off", self.set_animation_twinkle_off)
        wpilib.SmartDashboard.putData(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Animation", self._color_chooser)

    def periodic(self):
        if self._color_chooser.getSelected() != self.selected_animation:
            self.selected_animation = self._color_chooser.getSelected()
            self.selected_animation()

    # --------------------------
    # LED control functions - Red, Green, Blue, Alpha (transparency, usually 0 for full color)
    # --------------------------
    def set_all_leds_RGBW(self, red: int = 0, green: int = 0, blue: int = 0, white: int = 0):
        red = red or self.rgb_red
        green = green or self.rgb_green
        blue = blue or self.rgb_blue
        white = white or self.rgb_white
        self.candle.set_control(SolidColor(led_start_index=0, led_end_index=7, 
                                           color=RGBWColor(red, green, blue, white)))

    def set_animation_fire(self):
        self.candle.set_control(FireAnimation(led_start_index=0, led_end_index=7))

    def set_animation_twinkle(self, red: int = 0, green: int = 255, blue: int = 0, white: int = 0):
        red = red or self.rgb_red
        green = green or self.rgb_green
        blue = blue or self.rgb_blue
        white = white or self.rgb_white
        self.candle.set_control(TwinkleAnimation(led_start_index=0, led_end_index=7, 
                                                 color=RGBWColor(red, green, blue, white)))

    def set_animation_single_fade(self, red: int, green: int, blue: int, white: int = 0, frame_rate: int = 30):
        red = red or self.rgb_red
        green = green or self.rgb_green
        blue = blue or self.rgb_blue
        white = white or self.rgb_white
        self.candle.set_control(SingleFadeAnimation(led_start_index=0, led_end_index=7, 
                             color=RGBWColor(red, green, blue, white), frame_rate=frame_rate))

    def set_animation_larson(self, red: int, green: int, blue: int, white: int = 0, frame_rate: int = 30):
        red = red or self.rgb_red
        green = green or self.rgb_green
        blue = blue or self.rgb_blue
        white = white or self.rgb_white
        self.candle.set_control(LarsonAnimation(led_start_index=0, led_end_index=7, 
                                               color=RGBWColor(red, green, blue, white), 
                                               frame_rate=frame_rate))
        
    def set_animation_color_flow(self, red: int = 0, green: int = 0, blue: int = 0, white: int = 0, frame_rate: int = 30):
        red = red or self.rgb_red
        green = green or self.rgb_green
        blue = blue or self.rgb_blue
        white = white or self.rgb_white
        self.candle.set_control(ColorFlowAnimation(led_start_index=0, led_end_index=7, 
                                                   color=RGBWColor(red, green, blue, white), 
                                                   frame_rate=frame_rate))

    def set_animation_twinkle_off(self, red: int = 0, green: int = 0, blue: int = 0, white: int = 0):
        red = red or self.rgb_red
        green = green or self.rgb_green
        blue = blue or self.rgb_blue
        white = white or self.rgb_white
        self.candle.set_control(TwinkleOffAnimation(led_start_index=0, led_end_index=7,
                                                    color=RGBWColor(red, green, blue, white)))
