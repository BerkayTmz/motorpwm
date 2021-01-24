import serial
from detector import Detector
import cv2
import keyboard
from time import sleep

serialPort = serial.Serial(port="/dev/ttyUSB0", baudrate=115200,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

serialString = ''                           # Used to hold data coming over UART

RF = 10
RR = 15
LF = 20
LR = 25
STOP = 0
SPEED = 255
SPEED_TURN = 45


def send(char: str):
    if  char == 'a':  # turn left
        drive = bytearray()
        drive.append(LF)
        drive.append(SPEED_TURN)
        serialPort.write(drive)

        drive = bytearray()
        drive.append(RF)
        drive.append(SPEED)
        serialPort.write(drive)
        print("turn left")

    elif char == 'd':  # turn right
        drive = bytearray()
        drive.append(LF)
        drive.append(SPEED)
        serialPort.write(drive)

        drive = bytearray()
        drive.append(RF)
        drive.append(SPEED_TURN)
        serialPort.write(drive)
        print("turn right")

    elif char == 'w':
        drive = bytearray()
        drive.append(LF)
        drive.append(SPEED)
        serialPort.write(drive)

        drive = bytearray()
        drive.append(RF)
        drive.append(SPEED)
        serialPort.write(drive)
        print('Drive forward')
    elif char == 's':
        drive = bytearray()
        drive.append(LR)
        drive.append(SPEED)
        serialPort.write(drive)

        drive = bytearray()
        drive.append(RR)
        drive.append(SPEED)
        serialPort.write(drive)
        print('Drive backwards')
    elif char == 'q':
        drive = bytearray()
        drive.append(LR)
        drive.append(STOP)
        serialPort.write(drive)

        drive = bytearray()
        drive.append(RF)
        drive.append(STOP)
        serialPort.write(drive)
        print("Stop")

def drive(char: str):
    send(char)
    sleep(0.1)
    send('q')

keyboard.add_hotkey('w', drive, args=('w'))
keyboard.add_hotkey('s', drive, args=('s'))
keyboard.add_hotkey('a', drive, args=('a'))
keyboard.add_hotkey('d', drive, args=('d'))
keyboard.add_hotkey('q', drive, args=('q'))


try:
    keyboard.wait()
except:
    drive('q')