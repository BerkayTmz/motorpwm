import serial
from serial.tools import list_ports
from typing import Dict, Tuple, List
import redis

client = redis.Redis()

RIGHT_MOTOR_FORWARD = 10
RIGHT_MOTOR_REVERSE = 15
LEFT_MOTOR_FORWARD = 20
LEFT_MOTOR_REVERSE = 25

DRIVE_SPEED = 145
STOP = 0
DRIVE_TURN_SPEED = 60
TURN_SPEED = 140
STOP_DIST = 500


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


def rush(center_x):
    center_x = int((center_x + 1) * 100)

    if(center_x < 80):  # turn left
        robotDrive("DRIVE_LEFT")

    elif(center_x > 120):  # turn right
        robotDrive("DRIVE_RIGHT")
    else:
        robotDrive("FORWARD")


def controller(center_x, min_fdist):
    if center_x and (min_fdist is None or min_fdist > STOP_DIST):
        rush(center_x)
    else:
        robotDrive('STOP')


while True:
    redis_vals = client.mget(['center:x', 'f:dist'])
    center_x, min_fdist = redis_vals[0], redis_vals[1]
    center_x = None if center_x is None else float(center_x)
    min_fdist = None if min_fdist is None else float(min_fdist)
    controller(center_x, min_fdist)
