import serial
from detector import Detector, TargetType
import argparse
import cv2
import numpy as np
from serial import Serial
import base64
import redis
import time
from enum import Enum, auto

client = redis.Redis()
pubsub = client.pubsub()

# search algorithm parameters
TURN_BASE_TIME = 0.4            # seconds
FORWARD_BASE_TIME = 1           # seconds
T = 1                           # unitless
stop_distance = 400             # millimeters
obstacle_avoided_dist = 1000    # millimeters

# Codes to communicate with Arduino to drive individual motors to each direction
RIGHT_MOTOR_FORWARD = 20
RIGHT_MOTOR_REVERSE = 25
LEFT_MOTOR_FORWARD = 10
LEFT_MOTOR_REVERSE = 15


# Pre-defined motor speeds
# Values must be between 0-255, this value configures to PWM pulse width
DRIVE_SPEED = 70
STOP = 0
DRIVE_TURN_SPEED = 45
TURN_SPEED = 80

# Search algorithm state data [DON'T MODIFY]
s_state_changed = True
s_state = 0
s_cntr = 1
s_start_time = time.time()
cooldown_until = time.time()

# Obstacle avoidance algorithm state data [DON'T MODIFY]
o_state_changed = True
o_state = 0
o_start_time = time.time()
o_down_start = time.time()
y = 0


class Mode(Enum):
    Rush = auto()
    Search = auto()
    ObstacleAvoidance = auto()


Mode = Mode.Search

parser = argparse.ArgumentParser()
parser.add_argument("-d", "--display", help="display camera feed",
                    action="store_true")
parser.add_argument("-s", "--serial", help="start serial communication with ESP",
                    action="store_true")
args = parser.parse_args()

serialPort = serial.Serial(port="/dev/ttyUSB0", baudrate=115200,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)


def robotDriveArduino(leftMotorDirection, leftMotorSpeed, rightMotorDirection, rightMotorSpeed):
    drive = bytearray()
    drive.append(leftMotorDirection)
    drive.append(leftMotorSpeed)
    serialPort.write(drive)

    drive = bytearray()
    drive.append(rightMotorDirection)
    drive.append(rightMotorSpeed)
    serialPort.write(drive)


def robotDrive(driveCondition):
    if(driveCondition == "DRIVE_LEFT"):
        # Turn left, slower left motor with turing speed and right motor with normal driving speed
        robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_TURN_SPEED,
                          RIGHT_MOTOR_FORWARD, DRIVE_SPEED)

    elif(driveCondition == "DRIVE_RIGHT"):
        # Turn right, slower right motor with turn speed and left motor with normal driving speed
        robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED,
                          RIGHT_MOTOR_FORWARD, DRIVE_TURN_SPEED)

    elif(driveCondition == "TURN_LEFT"):
        # Turn right, slower right motor with turn speed and left motor with normal driving speed
        robotDriveArduino(LEFT_MOTOR_REVERSE, TURN_SPEED,
                          RIGHT_MOTOR_FORWARD, TURN_SPEED)

    elif(driveCondition == "TURN_RIGHT"):
        # Turn right, slower right motor with turn speed and left motor with normal driving speed
        robotDriveArduino(LEFT_MOTOR_FORWARD, TURN_SPEED,
                          RIGHT_MOTOR_REVERSE, TURN_SPEED)

    elif(driveCondition == "FORWARD"):
        # Drive forward, both motors are driven with DRIVE_SPEED in forward direction
        robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED,
                          RIGHT_MOTOR_FORWARD, DRIVE_SPEED)

    elif(driveCondition == "REVERSE"):
        # Drive reverse, both motors are driven with DRIVE_SPEED in reverse direction
        robotDriveArduino(LEFT_MOTOR_REVERSE, DRIVE_SPEED,
                          RIGHT_MOTOR_REVERSE, DRIVE_SPEED)

    elif(driveCondition == "STOP"):
        # STOP, stop signal is send to moth motors
        robotDriveArduino(LEFT_MOTOR_FORWARD, STOP, RIGHT_MOTOR_FORWARD, STOP)

    else:
        robotDriveArduino(LEFT_MOTOR_FORWARD, STOP, RIGHT_MOTOR_FORWARD, STOP)


def init_mode_params(mode: str):
    if mode == "s":
        global s_state, s_state_changed, s_cntr
        s_state_changed = True
        s_state = 0
        s_cntr = 1

    elif mode == "o":
        global o_state, o_state_changed
        o_state_changed = True
        o_state = 0


def lidar_distance(direction: str):
    min_dist = client.get(f'{direction}:dist')
    min_dist = None if min_dist is None else float(min_dist)
    return min_dist


def cooldown(cd: float = None):
    global cooldown_until
    if cd is None:
        return (cooldown_until > time.time())
    else:
        cooldown_until = time.time() + cd


def nextState(mode: str):
    if mode == "s":
        global s_state, s_state_changed, s_cntr
        s_state += 1
        if s_state > 3:
            s_state = 0
            s_cntr += 1
        cooldown(1)
        s_state_changed = True
    elif mode == "o":
        global o_state, o_state_changed
        o_state += 1
        if o_state > 6:
            o_state = 0
        cooldown(1)
        o_state_changed = True


def rush(frame, target, center_x, center_y, min_dist):
    global stop_distance

    f = lidar_distance("f")
    if f and f < stop_distance:
        robotDrive("STOP")
        return
    elif center_x:
        center_x = int((center_x + 1)*100)
        center_y = int((center_y + 1)*100)

        # s_state_changed = 2
        # s_state = 0
        # s_cntr = 1
        # s_start_time = time.time()

        if center_y < 40:
            robotDrive("STOP")
            return
        else:
            if(center_x < 90):  # turn left
                # Turn left, slower left motor with turing speed and right motor with normal driving speed
                #robotDriveArduino(LEFT_MOTOR_FORWARD, TURN_SPEED, RIGHT_MOTOR_FORWARD, DRIVE_SPEED)
                robotDrive("DRIVE_LEFT")

            elif(center_x > 110):  # turn right
                # Turn right, slower right motor with turn speed and left motor with normal driving speed
                #robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED, RIGHT_MOTOR_FORWARD, TURN_SPEED)
                robotDrive("DRIVE_RIGHT")
            else:
                # Drive forward, this condition is only happens when the
                # robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED, RIGHT_MOTOR_FORWARD, DRIVE_SPEED)
                robotDrive("FORWARD")


def obstacleAvoidance():
    global o_state_changed, o_state, o_start_time, o_down_start, Mode, y

    first_entrance_for_this_state = False


    if cooldown():
        return
    
    # Execute obstacle avoidance algorithm
    if o_state_changed:
        o_start_time = time.time()
        o_state_changed = False
        first_entrance_for_this_state = True

    if o_state == 0:
        if time.time() < o_start_time + (TURN_BASE_TIME*T):
            robotDrive("TURN_LEFT")
        else:
            nextState("o")
            return

    elif o_state == 1:
        r = lidar_distance("r")
        # and time.time() < o_start_time + searchtengelen:
        if r and r < obstacle_avoided_dist:
            robotDrive("FORWARD")
            if first_entrance_for_this_state:
                o_down_start = time.time()
        else:
            nextState("o")
            y = time.time() - o_down_start
            return

    elif o_state == 2:
        if time.time() < o_start_time + (TURN_BASE_TIME*T):
            robotDrive("TURN_RIGHT")
        else:
            nextState("o")
            return
    
# Araya state ekle default bir süre gitsin (1 metre götürcek kadar mesela)
# x initialize et, yatay gittiğin yolu ölç [gerekirse]

    elif o_state == 3:
        r = lidar_distance("r")
        # and time.time() < o_start_time + searchtengelen:
        if r and r < obstacle_avoided_dist:
            robotDrive("FORWARD")
        else:
            nextState("o")
            return

    elif o_state == 4:
        if time.time() < o_start_time + (TURN_BASE_TIME*T):
            robotDrive("TURN_RIGHT")
        else:
            nextState("o")
            return

    elif o_state == 5:
        r = lidar_distance("r")
        # and time.time() < o_start_time + searchtengelen:
        if time.time() < o_start_time + y:
            robotDrive("FORWARD")
        else:
            nextState("o")
            return

    elif o_state == 6:
        if time.time() < o_start_time + (TURN_BASE_TIME*T):
            robotDrive("TURN_LEFT")
        else:
            nextState("o")
            Mode = Mode.Search
            init_mode_params("o")
            return



def search():
    global s_state_changed, s_state, s_cntr, s_start_time, stop_distance, Mode

    f = lidar_distance("f")
    if f and f < stop_distance:
        robotDrive("STOP")
        Mode = Mode.ObstacleAvoidance
        cooldown(1)
        return

    if cooldown():
        return

    # Execute search algorithm
    if s_state_changed:
        s_start_time = time.time()
        s_state_changed = False

    if s_state == 0:
        if time.time() < s_start_time + (s_cntr*FORWARD_BASE_TIME*T):
            robotDrive("FORWARD")
        else:
            nextState("s")
            return

    elif s_state == 1:
        if time.time() < s_start_time + (TURN_BASE_TIME*T):
            robotDrive("TURN_LEFT")
        else:
            nextState("s")
            return

    elif s_state == 2:
        if time.time() < s_start_time + (s_cntr*FORWARD_BASE_TIME*T):
            robotDrive("FORWARD")
        else:
            nextState("s")
            return

    elif s_state == 3:
        if time.time() < s_start_time + (TURN_BASE_TIME*T):
            robotDrive("TURN_LEFT")
        else:
            nextState("s")
            return


def controller(frame, target, center_x, center_y, min_dist):
    global Mode
    # Adjust Mode
    if center_x:
        Mode = Mode.Rush

    # Execute Mode
    if Mode == Mode.Rush:
        rush(frame, target, center_x, center_y, min_dist)
    elif Mode == Mode.Search:
        search()
    elif Mode == Mode.ObstacleAvoidance:
        obstacleAvoidance()
    else:
        robotDrive("STOP")


def serial_comm(frame):
    resized = cv2.resize(frame, (80, 60))
    retval, buf = cv2.imencode('.jpg', resized)
    content = buf.tobytes()
    to_write = base64.b64encode(content)

    client.set("serial_image", to_write)


with Detector(0, (640, 360)) as detector:
    target, target_type = None, None
    while True:
        frame, target, target_type, center_x, center_y = detector.detect(
            target, target_type)
        min_dist = client.get("min")
        min_dist = 0 if min_dist is None else float(min_dist)
        if args.serial:
            serial_comm(frame)
        if args.display:
            cv2.imshow("Display", frame)
            if cv2.waitKey(1) == 27:
                break
        controller(frame, target, center_x, center_y, min_dist)
