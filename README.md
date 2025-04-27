# Soccer Robot (Ball Finding)

This is a robot that uses L298n motor drivers to control four (4) DC motors, a 1080p web camera, a Raspberry Pi 5 8GB, 2x18650 batteries for motor power supply, and a powerbank for portable Raspberry Pi power supply. The code structure is presented below.

```bash
src/
├── ball_finder.py
├── crontab_script
├── detection.py
├── export.py
├── models
│   ├── y11ndetect_ncnn_model
│   │   ├── metadata.yaml
│   │   ├── model.ncnn.bin
│   │   ├── model.ncnn.param
│   │   └── model_ncnn.py
│   ├── y11ndetect.pt
│   └── y11ndetect.torchscript
├── motor_test.py
├── soccer_dynamic.py
├── socker_static.py
└── todo.txt
```

## Code

### `ball_finder.py`
> This is a perpetual ball finding script where the robot will register a specific ball (closest if multiple balls are present) in the allowed classes (blue, green, and pink). It will follow the ball until you turn off the robot or put a cover in the camera.

### `soccer_dynamic.py`
> This is a perpetual ball finding and goal finding script. The robot starts looking for the closest ball and registers its color. It then attempts to reach the ball, after reaching it, it will find the goal with the same corresponding color. Then, it will reach that goal. It will do the same process over and over again until you put a cover in the camera after it reaches a goal or you turn off the robot completely. 

### `soccer_static.py`
> This is the unsupported initial code. It uses a static speed for the motors and doesn't do a closed loop adjustment of the PWM values - which are used for adjusting motor speeds.

### `motor_test.py`
> This is a simple motor testing script.

### `export.py`
> Exports the model from .pt to .ncnn which provides fastest runtime in Raspberry Pi or ARM devices.

### `detection.py`
> A simple script that runs the model and shows the objects being detected by the camera. It shows the bounding box, how away it is from the center, the assumed area of the bounding box objects, etc.

### `models/`
> These are the models used in the model. These are finetuned YOLO11 nano models for object detection task. The .pt, ncnn, and .torchscript files are present here.

### `crontab_script`
> This is the crontab script that runs the code that runs automatically when the Raspberry Pi is booted. So, when you open your Raspberry Pi, the code present in the crontab script runs
```bash
@reboot /home/ball_finder/Scripts/env/bin/python /home/ball_finder/Scripts/src/soccer_dynamic.py
```
> `@reboot` - means that this code must be ran during startup or reboot

> `/home/ball_finder/Scripts/env/bin/python` - means that you are using this version of Python to run

> `/home/ball_finder/Scripts/src/soccer_dynamic.py` - this is the Python script you are running (which is the default)

## How do I change the script I run during runtime?

1. Open crontab by typing `crontab -e` in the terminal.
2. If you want to change from `soccer_dynamic.py` to `ball_finder.py` for example, just change the `soccer_dynamic.py` to `ball_finder.py`. As easy as that, take note that you can see the code way below the crontab script, you just have to change that.
3. So essentially you have the code below:
```bash
some_comments_on_top_that_starts_with_#
@reboot /home/ball_finder/Scripts/env/bin/python /home/ball_finder/Scripts/src/ball_finder.py
```
4. Type `CTRL+O` then `ENTER` then `CTRL+X`. This will save that.
5. To ensure you have the right script in your crontab, type `crontab -l` in the terminal. This will show you the code you wrote.