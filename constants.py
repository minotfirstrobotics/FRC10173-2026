SWERVE_DEFAULT_NOT_GENERATED = {
    "DEFAULT_AUTONOMOUS": "Tests",
    "MAX_ROTATION_SPEED": 0.75, # radians per second
    "MAX_POV_SPEED": 0.2, # percent
    "MAX_DRIVE_SPEED_FACTOR": .5
    }

DASHBOARD_TITLES = {
    "AUTONOMOUS_MODE": "Autonomous Mode",
    "GENERAL_MOTOR_RUNNING": "General Motor Running",
    "180_SERVO_POSITION": "180 Servo Position",
    "180_SERVO_DESTINATION": "General Servo Destination",
    "WINCH_SERVO_POSITION": "Winch Servo Position",
    "WINCH_SERVO_DESTINATION": "Winch Servo Destination",
    "ENCODED_MOTOR_RUNNING": "Encoded Motor Running",
    "ENCODED_MOTOR_POSITION": "Encoded Motor Position",
    "SHOOTER_EMOTOR_RUNNING": "Shooter EMotor Running",
    "SHOOTER_EMOTOR_POSITION": "Shooter EMotor Position",
}

PWM_CHANNELS = {
    "GENERAL_MOTOR": 0,
    "180_SERVO": 8,
    "WINCH_SERVO": 1,
}

CAN_CHANNELS = {
    "ENCODED_MOTOR": 13,
    "SHOOTER_EMOTOR": 14, #A number to be determined based on the actual CAN ID assigned to the motor controller
}


