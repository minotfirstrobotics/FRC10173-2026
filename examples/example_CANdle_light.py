from phoenix6.hardware import CANdle
from phoenix6.configs import CANdleConfiguration
from phoenix6.signals import StripTypeValue, RGBWColor
from phoenix6.controls import FireAnimation, SolidColor

class MyRobot:
    def robotInit(self):
        self.candle = CANdle(0) # check Phoenix Tuner X for the correct ID#

        # Fluent-style LED config
        configs = CANdleConfiguration()
        configs.led.with_strip_type(StripTypeValue.RGB).with_brightness_scalar(0.5)
        self.candle.configurator.apply(configs)

        # Create a control request to set the color
        # Red, Green, Blue, Alpha (transparency, usually 0 for full color)
        self.set_color_request = SolidColor(led_start_index=0, led_end_index=7)
        self.fire_request = FireAnimation(led_start_index=0, led_end_index=7)

    def teleopPeriodic(self):
        # Periodically set the color using a control request
        # The API is signal-based, so you "set" a control request as the device's output
        self.set_all_leds_RGB(red=255, green=0, blue=0) # Set all LEDs to red

    def set_all_leds_RGB(self, red: int = 0, green: int = 0, blue: int = 0, white: int = 0):
        # To turn off all LEDs, set the color to black (0,0,0) or disable them
        self.candle.set_control(self.set_color_request.with_color(RGBWColor(red, green, blue, white)))

    def set_all_leds_off(self):
        # To turn off all LEDs, set the color to black (0,0,0) or disable them
        self.candle.set_control(self.set_color_request.with_color(RGBWColor(0, 0, 0, 0)))

    def set_animation_fire(self):
        self.candle.set_control(self.fire_request)