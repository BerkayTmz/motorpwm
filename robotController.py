import serial
from detector import Detector, TargetType
import argparse
import cv2
from serial import Serial
import base64
import redis

client = redis.Redis()
pubsub = client.pubsub()

parser = argparse.ArgumentParser()
parser.add_argument("-d", "--display", help="display camera feed",
                    action="store_true")
parser.add_argument("-s", "--serial", help="start serial communication with ESP",
                    action="store_true")
args = parser.parse_args()

serialPort = serial.Serial(port="/dev/ttyUSB0", baudrate=115200,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

serialString = ""                           # Used to hold data coming over UART

# Codes to communicate with Arduino to drive individual motors to each direction
RIGHT_MOTOR_FORWARD = 10
RIGHT_MOTOR_REVERSE = 15
LEFT_MOTOR_FORWARD = 20
LEFT_MOTOR_REVERSE = 25


# Pre-defined motor speeds
# Values must be between 0-255, this value configures to PWM pulse width
DRIVE_SPEED = 100
STOP = 0
TURN_SPEED = 70


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
    if(driveCondition == "TURN_LEFT"):
        # Turn left, slower left motor with turing speed and right motor with normal driving speed
        robotDriveArduino(LEFT_MOTOR_FORWARD, TURN_SPEED,
                          RIGHT_MOTOR_FORWARD, DRIVE_SPEED)

    elif(driveCondition == "TURN_RIGHT"):
        # Turn right, slower right motor with turn speed and left motor with normal driving speed
        robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED,
                          RIGHT_MOTOR_FORWARD, TURN_SPEED)

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


def controller(frame, target, center_x, center_y, min_dist):

    if center_x and (min_dist == 0 or min_dist >= 300):
        center_x = int((center_x + 1)*100)
        center_y = int((center_y + 1)*100)

        if(center_y > 30):
            if(center_x < 90):  # turn left
                # Turn left, slower left motor with turing speed and right motor with normal driving speed
                #robotDriveArduino(LEFT_MOTOR_FORWARD, TURN_SPEED, RIGHT_MOTOR_FORWARD, DRIVE_SPEED)
                robotDrive("TURN_LEFT")

            elif(center_x > 110):  # turn right
                # Turn right, slower right motor with turn speed and left motor with normal driving speed
                #robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED, RIGHT_MOTOR_FORWARD, TURN_SPEED)
                robotDrive("TURN_RIGHT")
            else:
                # Drive forward, this condition is only happens when the
                # robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED, RIGHT_MOTOR_FORWARD, DRIVE_SPEED)
                robotDrive("FORWARD")
    else:
        # STOP, stop signal is send to moth motors
        # robotDriveArduino(LEFT_MOTOR_REVERSE, STOP, RIGHT_MOTOR_FORWARD, STOP)
        robotDrive("STOP")


def serial_comm(frame):
    resized = cv2.resize(frame, (80, 60))
    retval, buf = cv2.imencode('.jpg', resized)
    content = buf.tobytes()
    to_write = base64.b64encode(content)

    client.set("serial_image", to_write)


with Detector(0, (640, 480)) as detector:
    target, target_type = None, None
    min_dist = int(client.get("min"))
    while True:
        frame, target, target_type, center_x, center_y = detector.detect(
            target, target_type)
        if args.serial:
            serial_comm(frame)
        if args.display:
            cv2.imshow("Display", frame)
            if cv2.waitKey(1) == 27:
                break
        controller(frame, target, center_x, center_y, min_dist)
