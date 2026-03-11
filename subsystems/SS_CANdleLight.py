import commands2
from phoenix6.hardware import CANdle
from phoenix6.configs import CANdleConfiguration
from phoenix6.configs.config_groups import CANdleFeaturesConfigs
from phoenix6.signals import StripTypeValue, RGBWColor
from phoenix6.controls import FireAnimation, SingleFadeAnimation, TwinkleAnimation, LarsonAnimation, ColorFlowAnimation 
from phoenix6.controls import TwinkleOffAnimation, SolidColor

class SS_CANdleLight(commands2.Subsystem):
    def __init__(self, CANdle_channel: int):
        super().__init__()
        self.candle = CANdle(CANdle_channel)

        # Fluent-style LED config
        configs = CANdleConfiguration()
        configs.led.with_strip_type(StripTypeValue.RGB).with_brightness_scalar(0.5)
        configs.candle_features.with_enable5_v_rail(True)  # Enable 5V VOUT
        self.candle.configurator.apply(configs)
        self.set_all_leds_RGBW(red=0, green=0, blue=255) # Set all LEDs to blue

    def periodic(self):
        pass

    # --------------------------
    # LED control functions
    # --------------------------
    def set_all_leds_RGBW(self, red: int = 0, green: int = 0, blue: int = 0, white: int = 0):
        # Red, Green, Blue, Alpha (transparency, usually 0 for full color)
        self.candle.set_control(SolidColor(led_start_index=0, led_end_index=7, 
                                           color=RGBWColor(red, green, blue, white)))

    def set_animation_fire(self):
        self.candle.set_control(FireAnimation(led_start_index=0, led_end_index=7))

    def set_animation_twinkle(self):
        self.candle.set_control(TwinkleAnimation(led_start_index=0, led_end_index=7, 
                                                 color=RGBWColor(0, 255, 0, 0)))

    def set_animation_single_fade(self, red: int, green: int, blue: int, white: int = 0, frame_rate: int = 30):
        self.candle.set_control(SingleFadeAnimation(led_start_index=0, led_end_index=7, 
                             color=RGBWColor(red, green, blue, white), frame_rate=frame_rate))

    def set_animation_larson(self, red: int, green: int, blue: int, white: int = 0, frame_rate: int = 30):
        self.candle.set_control(LarsonAnimation(led_start_index=0, led_end_index=7, 
                                               color=RGBWColor(red, green, blue, white), 
                                               frame_rate=frame_rate))
        
    def set_animation_color_flow(self, frame_rate: int = 30):
        self.candle.set_control(ColorFlowAnimation(led_start_index=0, led_end_index=7, 
                                                   frame_rate=frame_rate))

    def set_animation_twinkle_off(self):
        self.candle.set_control(TwinkleOffAnimation(led_start_index=0, led_end_index=7))

    # --------------------------
    # Commands
    # --------------------------
    def set_LED_color_command(self, red: int, green: int, blue: int, white: int = 0):
        return commands2.cmd.runOnce(lambda: self.set_all_leds_RGBW(red, green, blue, white), self)
    
    def set_LED_off_command(self):
        return commands2.cmd.runOnce(lambda: self.set_all_leds_RGBW(red=0, green=0, blue=0, white=0), self)
        
    def set_fire_animation_command(self):
        return commands2.cmd.runOnce(lambda: self.set_animation_fire(), self)
    
    def set_twinkle_animation_command(self):
        return commands2.cmd.runOnce(lambda: self.set_animation_twinkle(), self)
    
    def set_animation_single_fade_command(self, red: int, green: int, blue: int, white: int = 0, frame_rate: int = 30):
        return commands2.cmd.runOnce(lambda: self.set_animation_single_fade(red, green, blue, white, frame_rate), self)
    
    def set_larson_animation_command(self, red: int, green: int, blue: int, white: int = 0, frame_rate: int = 30):
        return commands2.cmd.runOnce(lambda: self.set_animation_larson(red, green, blue, white, frame_rate), self)

    def set_color_flow_animation_command(self, frame_rate: int = 30):
        return commands2.cmd.runOnce(lambda: self.set_animation_color_flow(frame_rate), self)

    def set_twinkle_off_animation_command(self):
        return commands2.cmd.runOnce(lambda: self.set_animation_twinkle_off(), self)
