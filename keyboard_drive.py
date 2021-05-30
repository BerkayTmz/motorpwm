import serial
from detector import Detector
import cv2
import keyboard
from time import sleep
import argparse
from concurrent.futures import ProcessPoolExecutor
from serial.tools import list_ports
from typing import List, Tuple


def get_device_com(comports, vid_pid_tuple: List[Tuple[int, int]]):
    device_com = [com.device for com in comports
                  if (com.vid, com.pid) in vid_pid_tuple]
    return device_com[0] if len(device_com) else None


ARDU_VID_PID = [(1027, 24577), (9025, 66)]
comports = list(list_ports.comports())
ardu_dev_com = get_device_com(comports, ARDU_VID_PID)
serialPort = serial.Serial(port=ardu_dev_com, baudrate=115200,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

serialString = ''                           # Used to hold data coming over UART


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
      robotDriveArduino(LEFT_MOTOR_REVERSE, DRIVE_SPEED, RIGHT_MOTOR_FORWARD, DRIVE_SPEED)

   elif(driveCondition == "TURN_RIGHT"):
      # Turn right, slower right motor with turn speed and left motor with normal driving speed 
      robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED, RIGHT_MOTOR_REVERSE, DRIVE_SPEED)

   elif(driveCondition == "FORWARD"):
      # Drive forward, both motors are driven with DRIVE_SPEED in forward direction
      robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED, RIGHT_MOTOR_FORWARD, DRIVE_SPEED)

   elif(driveCondition == "REVERSE"):
      # Drive reverse, both motors are driven with DRIVE_SPEED in reverse direction 
      robotDriveArduino(LEFT_MOTOR_REVERSE, DRIVE_SPEED, RIGHT_MOTOR_REVERSE, DRIVE_SPEED)
      
   elif(driveCondition == "STOP"):
       #STOP, stop signal is send to moth motors  
      robotDriveArduino(LEFT_MOTOR_FORWARD, STOP, RIGHT_MOTOR_FORWARD, STOP)

   else:
      robotDriveArduino(LEFT_MOTOR_FORWARD, STOP, RIGHT_MOTOR_FORWARD, STOP)



def send(char: str):
    if  char == 'a':  # turn left
        robotDrive("TURN_LEFT")

    elif char == 'd':  # turn right
        robotDrive("TURN_RIGHT")

    elif char == 'w':
        robotDrive("FORWARD")

    elif char == 's':
        robotDrive("REVERSE")

    elif char == 'q':
        robotDrive("STOP")

def drive(char: str):
    send(char)
    sleep(0.1)
    send('q')

keyboard.add_hotkey('w', drive, args=('w'))
keyboard.add_hotkey('s', drive, args=('s'))
keyboard.add_hotkey('a', drive, args=('a'))
keyboard.add_hotkey('d', drive, args=('d'))
keyboard.add_hotkey('q', drive, args=('q'))

def keyboard_control():
    try:
        keyboard.wait()
    except:
        drive('q')

def display_camera_feed():
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        cv2.imshow("Display", frame)
        if cv2.waitKey(1) == 27:
            break

keyboard_control()
