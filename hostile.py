import serial
from serial.tools import list_ports
from typing import Dict, Tuple, List
import redis
import time
from enum import IntEnum


client = redis.Redis()

# Rush algorithm parameters
rush_stop_distance = 1000
center_x = 0
next_redis_read = 0
next_arduino_clear = 0
escape_params_initialized_time = 0

# escape algorithm parameters
TURN_BASE_TIME = 0.33                   # seconds
FORWARD_BASE_TIME = 0.5                 # seconds
T = 1                                   # unitless
obstacle_seen_stop_distance = 600       # millimeters
obstacle_avoided_dist = 1000            # millimeters
side_obstacle_seen_stop_distance = 300  # millimeters

escape_started = 0
mission_complete_time = 15              # seconds

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
DRIVE_SPEED = 120
STOP = 0
DRIVE_TURN_SPEED = 60
TURN_SPEED = 140

# Print interval
print_interval = 500
last_print_time = 0


# Search algorithm state data [DON'T MODIFY]
e_state_changed = True
e_state = 0
e_cntr = 1
e_start_time = time.time()
cooldown_until = time.time()

# Obstacle avoidance algorithm state data [DON'T MODIFY]
o_state_changed = True
o_state = 0
o_start_time = time.time()
o_down_start = time.time()
y = 0

f = [9999, 9999, 9999]
r = 0
RUSH_WAIT = False
RUSH_START = 0


class Mode(IntEnum):
    Rush = 0
    Escape = 1
    ObstacleAvoidance = 2


MODE = Mode.Rush

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
    # else:
    #    robotDriveArduino(LEFT_MOTOR_FORWARD, STOP, RIGHT_MOTOR_FORWARD, STOP)


def init_mode_params(mode: str):
    global e_state, e_state_changed, e_cntr, o_state, o_state_changed, TURN_BASE_TIME, T, escape_params_initialized_time, escape_started

    if (mode == "e") and (time.time() > escape_params_initialized_time + 4*(TURN_BASE_TIME*T)):
        escape_params_initialized_time = time.time()
        e_state_changed = True
        e_state = 0
        e_cntr = 1
        escape_started = time.time()

    elif mode == "o":
        o_state_changed = True
        o_state = 0


def maintenance_task():
    def redis_read():
        global f, r, center_x, next_redis_read
        # Update Camera
        center_x = client.get('center:x')
        center_x = None if center_x is None else float(center_x)

        # Update Lidar
        min_dist_f = client.mget(['fl:dist', 'f:dist', 'fr:dist'])
        for i, item in enumerate(min_dist_f):
            f[i] = 9999 if item is None else float(item)

        min_dist_r = client.get('r:dist')
        r = 0 if min_dist_r is None else float(min_dist_r)

        # Put timeout
        next_redis_read = time.time() + 0.05

    # Clear lastSent data to Arduino
    def clear_lastsent():
        global next_arduino_clear, lastSent
        lastSent = ""

        # Put timeout
        next_arduino_clear = time.time() + 1

    if time.time() >= next_redis_read:
        redis_read()

    if time.time() >= next_arduino_clear:
        clear_lastsent()


def cooldown(cd: float = None):
    global cooldown_until
    if cd is None:
        return (cooldown_until > time.time())
    else:
        cooldown_until = time.time() + cd


def nextState(mode: str, add_cooldown: bool = True, cooldown_amount: float = 1):
    global e_state, e_state_changed, e_cntr, o_state, o_state_changed, lidar_active, client
    if mode == "e":
        e_state += 1
        e_state_changed = True

    elif mode == "o":
        o_state += 1
        if o_state > 7:
            o_state = 0
        o_state_changed = True

    if add_cooldown:
        cooldown(cooldown_amount)

    lidar_active = True


def rush(center_x):
    global rush_stop_distance, f, MODE

    if (f[1] and f[1] < rush_stop_distance):
        robotDrive("STOP")
        return

    else:
        robotDrive("STOP")
        return


def obstacleAvoidance():
    global o_state_changed, o_state, o_start_time, o_down_start, MODE, y, obstacle_seen_stop_distance, lidar_active, f, r

    first_entrance_for_this_state = False

    if lidar_active:
        if (f[1] and f[1] < obstacle_seen_stop_distance) or (f[0] and f[0] < side_obstacle_seen_stop_distance) or (f[2] and f[2] < side_obstacle_seen_stop_distance):
            robotDrive("STOP")
            MODE = Mode.ObstacleAvoidance
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
            MODE = Mode.Search
            init_mode_params("o")
            return


def escape():
    global e_state_changed, e_state, e_cntr, e_start_time, obstacle_seen_stop_distance, MODE, f, r, escape_started, lidar_active

    if lidar_active and ( (f[1] and f[1] < obstacle_seen_stop_distance) or (f[0] and f[0] < side_obstacle_seen_stop_distance) or (f[2] and f[2] < side_obstacle_seen_stop_distance) ):
        robotDrive("STOP")
        MODE = Mode.ObstacleAvoidance
        init_mode_params("o")
        cooldown(1)
        return

    # Execute escape algorithm
    if e_state_changed:
        e_start_time = time.time()
        e_state_changed = False

    # Turn 180 degree
    elif e_state == 0:
        if time.time() < e_start_time + 2*(TURN_BASE_TIME*T):
            lidar_active = False
            robotDrive("TURN_LEFT")
        else:
            nextState("e", False)
            return

    # Go forward as if there is no tomorrow
    elif e_state == 1:
        robotDrive("FORWARD")
        return


def controller(center_x):
    global MODE

    # Adjust Mode
    if center_x:
        init_mode_params("e")
        MODE = Mode.Escape

    # Execute Mode
    if MODE == Mode.Rush:
        rush(center_x)
    elif MODE == Mode.Escape:
        escape()
    elif MODE == Mode.ObstacleAvoidance:
        obstacleAvoidance()
    else:
        robotDrive("STOP")


while True:
    maintenance_task()
    if (escape_started != 0) and (time.time() >= escape_started + mission_complete_time):
        if center_x:
            escape_started = 0
        robotDrive("STOP")
    else:
        controller(center_x)
