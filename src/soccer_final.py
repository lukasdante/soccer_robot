from dataclasses import dataclass, field
from typing import List
import RPi.GPIO as GPIO
import cv2
from ultralytics import YOLO
import os
from collections import namedtuple

GPIO.setmode(GPIO.BCM)

import atexit
atexit.register(GPIO.cleanup)

BoundingBox = namedtuple('BoundingBox', ['x1', 'y1', 'x2', 'y2'])

@dataclass
class Motor:
    name: str
    enable_pin: int
    forward_pin: int
    backward_pin: int
    pwm_hz: int
    default_speed: float
    
    def __post_init__(self):
        self.speed = self.default_speed
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.forward_pin, GPIO.OUT)
        GPIO.setup(self.backward_pin, GPIO.OUT)
        self.pwm_controller = GPIO.PWM(self.enable_pin, self.pwm_hz)
        self.pwm_controller.start(int(self.speed))

    def move(self, direction=1):
        # TODO: create an easing function

        if direction > 0:
            # move forward
            GPIO.output(self.forward_pin, GPIO.HIGH)
            GPIO.output(self.backward_pin, GPIO.LOW)
        elif direction < 0:
            # move backward
            GPIO.output(self.forward_pin, GPIO.LOW)
            GPIO.output(self.backward_pin, GPIO.HIGH)
        else:
            # stop
            GPIO.output(self.forward_pin, GPIO.LOW)
            GPIO.output(self.backward_pin, GPIO.LOW)

    def __str__(self):
        return f'{self.name}: EN {self.enable_pin} - IN1 {self.forward_pin} - IN2 {self.backward_pin} - HZ {self.pwm_hz} - V {self.default_speed}'

class Detector:
    def __init__(self, model, confidence, cam_width, cam_height):
        os.environ['YOLO_VERBOSE']='False'
        self.model = YOLO(f"{model}", verbose=False, task='detect')
        self.confidence = confidence
        self.connect_camera(cam_width, cam_height)
        self.center = (cam_width/2, cam_height/2)
        
    def connect_camera(self, cam_width, cam_height):
        self.cap = cv2.VideoCapture(-1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_height)

    def detect(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        results = self.model(frame, conf=self.confidence,)[0]

        areas_and_boxes = {}
        for box in results.boxes:
            label = self.model.names[int(box.cls)]
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            bbox = BoundingBox(x1, y1, x2, y2)
            # area = abs(x2 - x1) * abs(y2 - y1) # TODO: check if this is correct algorithm for area
            area = max(abs(y2 - y1), abs(x2 - x1)) ** 2
            areas_and_boxes[label] = [area, bbox]
        return areas_and_boxes

@dataclass
class Robot:
    name: str
    detector: Detector
    front_left: Motor
    front_right: Motor
    back_left: Motor
    back_right: Motor
    balls_left: List[str] = field(default_factory=lambda: ['blue_ball', 'green_ball', 'red_ball'])
    center_threshold: int = 1
    max_speed: int = 70
    base_speed: int = 20
    close_loop_gain: float = 0.25
    left_error: float = 0
    right_error: float = 0

    def move_sync(self, directions):
        self.front_left.move(directions[0])
        self.front_right.move(directions[1])
        self.back_left.move(directions[2])
        self.back_right.move(directions[3])

    def stop(self):
        self.move_sync([0, 0, 0, 0])

    def find_target(self, object):
        areas_and_boxes = self.detector.detect()
        target_areas = {k: v[1] for k, v in areas_and_boxes.items() if object in k}
        
        if 'ball' in object and target_areas:
            closest_ball = list(target_areas.keys())[0]
            print(f"We found all the balls, closest is {closest_ball}.")

            target_color = closest_ball.split("_")[0]
            print(f"Target color is set to {target_color}.")

            return target_color
    
    def move_dynamic(self, dif, area):
        self.close_loop_gain = 1.05*(area/15000)
        new_speed = min(self.max_speed, self.base_speed + abs(dif)*self.close_loop_gain); print("Correction: ", new_speed)

        if dif < 0:
            self.front_left.pwm_controller.ChangeDutyCycle(new_speed-5)
            self.back_left.pwm_controller.ChangeDutyCycle(new_speed-5)
            self.front_right.pwm_controller.ChangeDutyCycle(self.base_speed)
            self.back_right.pwm_controller.ChangeDutyCycle(self.base_speed)
            print(f"Velocity:   L:{new_speed}   R: {self.base_speed}")
        elif dif > 0:
            self.front_left.pwm_controller.ChangeDutyCycle(self.base_speed-5)
            self.back_left.pwm_controller.ChangeDutyCycle(self.base_speed-5)
            self.front_right.pwm_controller.ChangeDutyCycle(new_speed)
            self.back_right.pwm_controller.ChangeDutyCycle(new_speed)
            print(f"Velocity:   L:{self.base_speed}   R: {new_speed}")

        self.move_sync([1, 1, 1, 1])

    def follow_target(self, object):
        certainty = 0

        while True:
            areas_and_boxes = self.detector.detect()

            if object in areas_and_boxes:
                certainty = 0
                area, bbox = areas_and_boxes[object]
                x1, x2 = bbox.x1, bbox.x2
                ldx = abs(x1 - self.detector.center[0])
                rdx = abs(x2 - self.detector.center[0])
                dif = ldx - rdx
                print(f"DIF: {dif}  area: {area}")
                
                self.move_dynamic(dif, area)

            else:
                certainty += 1
                if certainty > 2:
                    self.stop()

def main():

    MOTOR_CONFIGS = {
        'front_left': {
            'enable_pin': 13,
            'forward_pin': 21,
            'backward_pin': 17
            },
        'front_right': {
            'enable_pin': 12,
            'forward_pin': 16,
            'backward_pin': 20
            },
        'back_left': {
            'enable_pin': 18,
            'forward_pin': 22,
            'backward_pin': 23
            },
        'back_right': {
            'enable_pin': 19,
            'forward_pin': 24,
            'backward_pin': 25
            }
    }

    DEFAULT_CONFIGS = {"pwm_hz": 1000, "default_speed": 50}

    # add default config to each motor config
    for motor_name in MOTOR_CONFIGS.keys():
        for parameter, value in DEFAULT_CONFIGS.items():
            MOTOR_CONFIGS[motor_name][parameter] = value

    # create a list of motors with order: fl, fr, bl, br
    motors = {}
    for motor_name, motor_config in MOTOR_CONFIGS.items():
        motor = Motor(name=motor_name, **motor_config)
        motors[motor_name] = motor

    detector = Detector(
        model="/home/ball_finder/Scripts/src/models/y11ndetect_ncnn_model",
        confidence=0.85,
        cam_width=640,
        cam_height=640,
    )

    robot = Robot(name="ball_finder", detector=detector, **motors)

    # go to the ball
    target_color = None
    while target_color is None:
        target_color = robot.find_target('ball')
    robot.follow_target(f'{target_color}_ball')


if __name__ == "__main__":
    main()