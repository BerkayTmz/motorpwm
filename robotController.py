import serial
from detector import Detector
import cv2

serialPort = serial.Serial(port = "/dev/ttyUSB0", baudrate=115200,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

serialString = ""                           # Used to hold data coming over UART

# Codes to communicate with Arduino to drive individual motors to each direction 
RIGHT_MOTOR_FORWARD = 10
RIGHT_MOTOR_REVERSE = 15
LEFT_MOTOR_FORWARD = 20
LEFT_MOTOR_REVERSE = 25


# Pre-defined motor speeds
# Values must be between 0-255, this value configures to PWM pulse width 
DRIVE_SPEED = 90
STOP = 0
TURN_SPEED = 45

def controller():
   with Detector(0, None) as d:
      target = None
      while True:
         frame, target, center_x, center_y = d.detect(target)

         if center_x:
            center_x = int((center_x + 1)*100)
            center_y = int((center_y + 1)*100)
            
            if(center_y > 30):
               if(center_x < 90): #turn left 
                  # Turn left, slower left motor with turing speed and right motor with normal driving speed
                  #robotDriveArduino(LEFT_MOTOR_FORWARD, TURN_SPEED, RIGHT_MOTOR_FORWARD, DRIVE_SPEED)
                  robotDrive("TURN_LEFT")

               elif(center_x > 110 ): # turn right
                  # Turn right, slower right motor with turn speed and left motor with normal driving speed 
                  #robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED, RIGHT_MOTOR_FORWARD, TURN_SPEED)
                  robotDrive("TURN_RIGHT")
               else:
                  # Drive forward, this condition is only happens when the 
                  # robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED, RIGHT_MOTOR_FORWARD, DRIVE_SPEED)
                  robotDrive("DRIVE_FORWARD")
         else: 
            # STOP, stop signal is send to moth motors  
            # robotDriveArduino(LEFT_MOTOR_REVERSE, STOP, RIGHT_MOTOR_FORWARD, STOP)
            robotDrive("STOP")
            
def robotDriveArduino(leftMotorDirection, leftMotorSpeed, rightMotorDirection, rightMotorSpeed):
   drive = bytearray()
   drive.append(leftMotorDirection)
   drive.append(leftMotorSpeed)
   serialPort.write(drive)

   drive = bytearray()
   drive.append(rightMotorDirection)
   drive.append(rightMotorSpeed)
   serialPort.write(drive)
   print(f"Center: ({center_x}, {center_y})") 


def robotDrive(driveCondition):
   if(driveCondition == "TURN_LEFT"):
      # Turn left, slower left motor with turing speed and right motor with normal driving speed
      robotDriveArduino(LEFT_MOTOR_FORWARD, TURN_SPEED, RIGHT_MOTOR_FORWARD, DRIVE_SPEED)

   elif(driveCondition == "TURN_RIGHT"):
      # Turn right, slower right motor with turn speed and left motor with normal driving speed 
      robotDriveArduino(LEFT_MOTOR_FORWARD, DRIVE_SPEED, RIGHT_MOTOR_FORWARD, TURN_SPEED)

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





while(True):
   controller()
   
   


