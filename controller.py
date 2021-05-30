import serial
from detector import Detector, TargetType
import argparse
import cv2
import numpy as np
from serial import Serial
from serial.tools import list_ports
from typing import Dict, Tuple, List
import base64
import redis
import time
from enum import Enum, auto


client = redis.Redis()
# pubsub = client.pubsub()

# Rush algorithm parameters
rush_stop_distance = 550

# search algorithm parameters
TURN_BASE_TIME = 0.33                  # seconds
FORWARD_BASE_TIME = 0.5                 # seconds
T = 1                                  # unitless
obstacle_seen_stop_distance = 700      # millimeters
obstacle_avoided_dist = 1000           # millimeters
side_obstacle_seen_stop_distance = 350  # millimeters

# Obctacle avoidance algorithm parameters
lidar_active = True


# Codes to communicate with Arduino to drive individual motors to each direction
RIGHT_MOTOR_FORWARD = 10
RIGHT_MOTOR_REVERSE = 15
LEFT_MOTOR_FORWARD = 20
LEFT_MOTOR_REVERSE = 25

lastSent = ""

# Pre-defined motor speeds
# Values must be between 0-255, this value configures to PWM pulse width
DRIVE_SPEED = 145
STOP = 0
DRIVE_TURN_SPEED = 60
TURN_SPEED = 140

# Print interval
print_interval = 500
last_print_time = 0

# Lidar interval
next_lidar_sampling_time = 0

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

# Auto Serial Port Finding


def get_device_com(comports, vid_pid_tuple: List[Tuple[int, int]]):
    device_com = [com.device for com in comports
                  if (com.vid, com.pid) in vid_pid_tuple]
    return device_com[0] if len(device_com) else None


ARDU_VID_PID = [(1027, 24577), (9025, 66)]
comports = list(list_ports.comports())
ardu_dev_com = get_device_com(comports, ARDU_VID_PID)
serialPort = serial.Serial(port=ardu_dev_com, baudrate=115200,
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
    global lastSent
    if(driveCondition == "DRIVE_LEFT" and lastSent != "DL"):
        # Turn left, slower left motor with turing speed and right motor with normal driving speed
        robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_TURN_SPEED,
                          RIGHT_MOTOR_FORWARD, DRIVE_SPEED)
        lastSent = "DL"                  

    elif(driveCondition == "DRIVE_RIGHT" and lastSent != "DR"):
        # Turn right, slower right motor with turn speed and left motor with normal driving speed
        robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED,
                          RIGHT_MOTOR_FORWARD, DRIVE_TURN_SPEED)
        lastSent = "DR"

    elif(driveCondition == "TURN_LEFT" and lastSent != "TL"):
        # Turn right, slower right motor with turn speed and left motor with normal driving speed
        robotDriveArduino(LEFT_MOTOR_REVERSE, TURN_SPEED,
                          RIGHT_MOTOR_FORWARD, TURN_SPEED)
        lastSent = "TL"

    elif(driveCondition == "TURN_RIGHT" and lastSent != "TR"):
        # Turn right, slower right motor with turn speed and left motor with normal driving speed
        robotDriveArduino(LEFT_MOTOR_FORWARD, TURN_SPEED,
                          RIGHT_MOTOR_REVERSE, TURN_SPEED)
        lastSent = "TR"

    elif(driveCondition == "FORWARD" and lastSent != "FF"):
        # Drive forward, both motors are driven with DRIVE_SPEED in forward direction
        robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED,
                          RIGHT_MOTOR_FORWARD, DRIVE_SPEED)
        lastSent = "FF"

    elif(driveCondition == "REVERSE" and lastSent != "RR"):
        # Drive reverse, both motors are driven with DRIVE_SPEED in reverse direction
        robotDriveArduino(LEFT_MOTOR_REVERSE, DRIVE_SPEED,
                          RIGHT_MOTOR_REVERSE, DRIVE_SPEED)
        lastSent = "RR"

    elif(driveCondition == "STOP" and lastSent != "SS"):
        # STOP, stop signal is send to moth motors
        robotDriveArduino(LEFT_MOTOR_FORWARD, STOP, RIGHT_MOTOR_FORWARD, STOP)
        lastSent = "SS"
    #else:
    #    robotDriveArduino(LEFT_MOTOR_FORWARD, STOP, RIGHT_MOTOR_FORWARD, STOP)


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


def lidar_distance(direction: bool):
    global next_lidar_sampling_time

    if direction:
        ret = [9999, 9999, 9999]
        if time.time() >= next_lidar_sampling_time:
            min_dist = client.mget(['fl:dist', 'f:dist', 'fr:dist'])
            for i, item in enumerate(min_dist):
                ret[i] = 9999 if item is None else float(item)
            next_lidar_sampling_time = time.time() + 0.1
        return ret

    else:
        min_dist = 9999
        if time.time() >= next_lidar_sampling_time:
            min_dist = client.get('r:dist')
            min_dist = None if min_dist is None else float(min_dist)
            next_lidar_sampling_time = time.time() + 0.1
        return min_dist


def cooldown(cd: float = None):
    global cooldown_until
    if cd is None:
        return (cooldown_until > time.time())
    else:
        cooldown_until = time.time() + cd


def nextState(mode: str, add_cooldown: bool = True, cooldown_amount: float = 1):
    if mode == "s":
        global s_state, s_state_changed, s_cntr
        s_state += 1
        if s_state > 3:
            s_state = 0
            s_cntr += 1
        s_state_changed = True
    elif mode == "o":
        global o_state, o_state_changed
        o_state += 1
        if o_state > 7:
            o_state = 0
        o_state_changed = True

    if add_cooldown:
        cooldown(cooldown_amount)

    global lidar_active
    lidar_active = True


def rush(center_x, center_y):
    global rush_stop_distance

    f = lidar_distance(True)
    if (f[1] and f[1] < rush_stop_distance) or not center_x:
        robotDrive("STOP")
        return
    elif center_x:
        center_x = int((center_x + 1)*100)

        if center_y:
            center_y = int((center_y + 1)*100)
            if center_y < 40:
                robotDrive("STOP")
                return

        # s_state_changed = 2
        # s_state = 0
        # s_cntr = 1
        # s_start_time = time.time()

        if(center_x < 80):  # turn left
            # Turn left, slower left motor with turing speed and right motor with normal driving speed
            #robotDriveArduino(LEFT_MOTOR_FORWARD, TURN_SPEED, RIGHT_MOTOR_FORWARD, DRIVE_SPEED)
            robotDrive("DRIVE_LEFT")

        elif(center_x > 120):  # turn right
            # Turn right, slower right motor with turn speed and left motor with normal driving speed
            #robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED, RIGHT_MOTOR_FORWARD, TURN_SPEED)
            robotDrive("DRIVE_RIGHT")
        else:
            # Drive forward, this condition is only happens when the
            # robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED, RIGHT_MOTOR_FORWARD, DRIVE_SPEED)
            robotDrive("FORWARD")


def obstacleAvoidance():
    global o_state_changed, o_state, o_start_time, o_down_start, Mode, y, obstacle_seen_stop_distance, lidar_active

    first_entrance_for_this_state = False

    if lidar_active:
        f = lidar_distance(True)
        if (f[1] and f[1] < obstacle_seen_stop_distance) or (f[0] and f[0] < side_obstacle_seen_stop_distance) or (f[2] and f[2] < side_obstacle_seen_stop_distance):
            robotDrive("STOP")
            Mode = Mode.ObstacleAvoidance
            lidar_active = False
            init_mode_params("o")
            cooldown(1)
            return

    if cooldown():
        robotDrive("STOP")
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
        r = lidar_distance(False)
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
        if time.time() < o_start_time + FORWARD_BASE_TIME*T:
            robotDrive("FORWARD")
        else:
            nextState("o", False)
            return

    elif o_state == 4:
        r = lidar_distance(False)
        # and time.time() < o_start_time + searchtengelen:
        if r and r < obstacle_avoided_dist:
            robotDrive("FORWARD")
        else:
            nextState("o")
            return

    elif o_state == 5:
        if time.time() < o_start_time + (TURN_BASE_TIME*T):
            robotDrive("TURN_RIGHT")
        else:
            nextState("o")
            return

    elif o_state == 6:
        r = lidar_distance(False)
        # and time.time() < o_start_time + searchtengelen:
        if time.time() < o_start_time + y:
            robotDrive("FORWARD")
        else:
            nextState("o")
            return

    elif o_state == 7:
        if time.time() < o_start_time + (TURN_BASE_TIME*T):
            robotDrive("TURN_LEFT")
        else:
            nextState("o")
            Mode = Mode.Search
            init_mode_params("o")
            return


def search():
    global s_state_changed, s_state, s_cntr, s_start_time, obstacle_seen_stop_distance, Mode

    f = lidar_distance(True)
    if (f[1] and f[1] < obstacle_seen_stop_distance) or (f[0] and f[0] < side_obstacle_seen_stop_distance) or (f[2] and f[2] < side_obstacle_seen_stop_distance):
        robotDrive("STOP")
        Mode = Mode.ObstacleAvoidance
        init_mode_params("o")
        cooldown(1)
        return

    if cooldown():
        robotDrive("STOP")
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


def controller(center_x=None, center_y=None):
    global Mode
    # Adjust Mode
    if center_x:
        Mode = Mode.Rush

    # Execute Mode
    if Mode == Mode.Rush:
        rush(center_x, center_y)
    elif Mode == Mode.Search:
        search()
    elif Mode == Mode.ObstacleAvoidance:
        obstacleAvoidance()
    else:
        robotDrive("STOP")


while True:

    # UNCOMMENT FOR TARGET/PEER IDENTIFICATION
    # center_x, center_y = client.get("center:x"), client.get("center:y")

    # center_x = None if center_x is None else float(center_x)
    # center_y = None if center_y is None else float(center_y)

    # Use this while being able to see the screen for debugging purposes
    # print("123")
    # current_loop_time = time.time()
    # if current_loop_time >= last_print_time + print_interval:
    #     last_print_time = current_loop_time
    #     print(str(Mode) + "   s_state: " + str(s_state) + "   o_state: " + str(o_state) + "    f: " + str(lidar_distance("f")) + "    r: " + str(lidar_distance("r")) + "    fl: " + str(lidar_distance("fl")) + "    fr: " + str(lidar_distance("fr")) + "     ", end="\r")
    # if center_x is not None and center_y is None:
    #     print(f"{center_x:0.3f} SICTIIIN             ", end="\r")
    # else:
    #     print(f"{None}              ", end="\r")

    # if center_y is not None:
    #     print(f"{center_y:0.3f}              ", end="\r")
    # else:
    #     print(f"{None}              ", end="\r")

    # controller(center_x, center_y)
    controller()
