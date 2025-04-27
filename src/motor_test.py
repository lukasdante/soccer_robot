import RPi.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

import atexit
atexit.register(GPIO.cleanup)

# Define motor pin mappings
motors = {
    'front_left': {'ENA': 12, 'IN1': 16, 'IN2': 20},
    'front_right': {'ENB': 13, 'IN3': 21, 'IN4': 17},
    'back_left': {'ENA': 18, 'IN1': 22, 'IN2': 23},
    'back_right': {'ENB': 19, 'IN3': 24, 'IN4': 25}
}

# Setup GPIO pins
for motor in motors.values():
    for pin in motor.values():
        GPIO.setup(pin, GPIO.OUT)

# Create PWM objects
pwm_FL = GPIO.PWM(motors['front_left']['ENA'], 1000)
pwm_FR = GPIO.PWM(motors['front_right']['ENB'], 1000)
pwm_BL = GPIO.PWM(motors['back_left']['ENA'], 1000)
pwm_BR = GPIO.PWM(motors['back_right']['ENB'], 1000)

# Start PWM with 50% duty cycle
pwm_FL.start(100)
pwm_FR.start(100)
pwm_BL.start(100)
pwm_BR.start(100)

def set_motor_direction(name, direction):
    motor = motors[name]
    if 'ENA' in motor:
        IN1, IN2 = motor['IN1'], motor['IN2']
    else:
        IN1, IN2 = motor['IN3'], motor['IN4']

    if direction == 'forward':
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    elif direction == 'backward':
        
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    else:  # stop
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)

def move_motors(motor_group, direction, duration):
    for name in motor_group:
        set_motor_direction(name, direction)
    print(f"{' & '.join(motor_group).replace('_', ' ').title()} moving {direction}")
    time.sleep(duration)
    for name in motor_group:
        set_motor_direction(name, 'stop')

try:
    # Sabay na forward ang front motors for 2 seconds
    move_motors(['front_left', 'front_right'], 'forward', 1)
    time.sleep(1)

    # Sabay na backward ang front motors for 2 seconds
    pwm_FL.ChangeDutyCycle(80)
    pwm_FR.ChangeDutyCycle(80)
    move_motors(['front_left', 'front_right'], 'backward', 1)
    time.sleep(1)

    # Sabay na forward ang back motors for 2 seconds
    move_motors(['back_left', 'back_right'], 'forward', 1)
    time.sleep(1)

    # Sabay na backward ang back motors for 2 seconds
    move_motors(['back_left', 'back_right'], 'backward', 1)
    time.sleep(1)

except:
    pass
