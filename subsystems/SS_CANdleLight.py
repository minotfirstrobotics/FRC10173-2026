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
        self.supply_battery_voltage = 0.0
        self.number_of_leds_to_light = 8
        self.candle = CANdle(CANdle_channel, canbus)
        self.dashboard_name = dashboard_name
        configs = CANdleConfiguration()
        configs.led.with_strip_type(StripTypeValue.RGB).with_brightness_scalar(0.5)
        self._periodic_counter = 0

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
        wpilib.SmartDashboard.putData(f"SS_Telemetry/CANDle {self.dashboard_name}/{self.dashboard_name} CANDle Animation", self._color_chooser)
        self.selected_animation = alliance_color_solid

    def periodic(self):
        self._periodic_counter += 1
        if self._periodic_counter % 5 == 0:  # Every 100ms instead of every 20ms
            if self._color_chooser.getSelected() != self.selected_animation:
                self.selected_animation = self._color_chooser.getSelected()
                self.selected_animation()
            rgb_red = wpilib.SmartDashboard.getNumber(f"SS_Telemetry/CANDle {self.dashboard_name}/{self.dashboard_name} CANDle RGB Red", self.rgb_red)
            rgb_green = wpilib.SmartDashboard.getNumber(f"SS_Telemetry/CANDle {self.dashboard_name}/{self.dashboard_name} CANDle RGB Green", self.rgb_green)
            rgb_blue = wpilib.SmartDashboard.getNumber(f"SS_Telemetry/CANDle {self.dashboard_name}/{self.dashboard_name} CANDleRGB Blue", self.rgb_blue)
            rgb_white = wpilib.SmartDashboard.getNumber(f"SS_Telemetry/CANDle {self.dashboard_name}/{self.dashboard_name} CANDleRGB White", self.rgb_white)
            if rgb_red != self.rgb_red or rgb_green != self.rgb_green or rgb_blue != self.rgb_blue or rgb_white != self.rgb_white:
                self.rgb_red = rgb_red
                self.rgb_green = rgb_green
                self.rgb_blue = rgb_blue
                self.rgb_white = rgb_white
                if self.selected_animation is not self.set_animation_fire:
                    self.selected_animation(red=rgb_red, green=rgb_green, blue=rgb_blue, white=rgb_white)

            ## Update the number of LEDs to light based on the supply voltage, with a deadzone to prevent flickering around threshold values
            present_voltage = round(self.candle.get_supply_voltage().value * 2) / 2 # Round to nearest 0.5V for stability in LED count changes 
            if present_voltage != self.supply_battery_voltage:
                self.supply_battery_voltage = present_voltage
                self.number_of_leds_to_light = int((present_voltage - 8.5) * 2)
                self.set_all_leds_RGBW(red=0, green=0, blue=0, white=0)  # Clear LEDs before applying new animation settings
                self.selected_animation()  # Update the animation to reflect the new number of LEDs to light

    # --------------------------
    # LED control functions - Red, Green, Blue, Alpha (transparency, usually 0 for full color)
    # --------------------------
    def _resolve_color(self, red: int = None, green: int = None, blue: int = None, white: int = None) -> RGBWColor:
        return RGBWColor(
            red if red is not None else self.rgb_red,
            green if green is not None else self.rgb_green,
            blue if blue is not None else self.rgb_blue,
            white if white is not None else self.rgb_white,
        )

    def set_all_leds_RGBW(self, red: int = None, green: int = None, blue: int = None, white: int = None):
        self.candle.set_control(SolidColor(led_start_index=0, led_end_index=self.number_of_leds_to_light, 
                                                    color=self._resolve_color(red, green, blue, white)))
    
    def set_animation_fire(self):
        self.candle.set_control(FireAnimation(led_start_index=0, led_end_index=self.number_of_leds_to_light))
    
    def set_animation_twinkle(self, red: int = None, green: int = None, blue: int = None, white: int = None):
        self.candle.set_control(TwinkleAnimation(led_start_index=0, led_end_index=self.number_of_leds_to_light, 
                                                    color=self._resolve_color(red, green, blue, white)))
    
    def set_animation_single_fade(self, red: int = None, green: int = None, blue: int = None, white: int = None, frame_rate: int = 30):
        self.candle.set_control(SingleFadeAnimation(led_start_index=0, led_end_index=self.number_of_leds_to_light, 
                                                    color=self._resolve_color(red, green, blue, white), frame_rate=frame_rate))
    
    def set_animation_larson(self, red: int = None, green: int = None, blue: int = None, white: int = None, frame_rate: int = 30):
        self.candle.set_control(LarsonAnimation(led_start_index=0, led_end_index=self.number_of_leds_to_light, 
                                                    color=self._resolve_color(red, green, blue, white), frame_rate=frame_rate))
    
    def set_animation_color_flow(self, red: int = None, green: int = None, blue: int = None, white: int = None, frame_rate: int = 30):
        self.candle.set_control(ColorFlowAnimation(led_start_index=0, led_end_index=self.number_of_leds_to_light, 
                                                    color=self._resolve_color(red, green, blue, white), frame_rate=frame_rate))
    
    def set_animation_twinkle_off(self, red: int = None, green: int = None, blue: int = None, white: int = None):
        self.candle.set_control(TwinkleOffAnimation(led_start_index=0, led_end_index=self.number_of_leds_to_light, 
                                                    color=self._resolve_color(red, green, blue, white)))