import serial
from detector import Detector
import cv2

serialPort = serial.Serial(port = "/dev/ttyUSB0", baudrate=115200,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

serialString = ""                           # Used to hold data coming over UART

RF = 10
RR = 15
LF = 20
LR = 25

while(1):
   # Wait until there is data waiting in the serial buffer
   # Print the contents of the serial data
   # Tell the device connected over the serial port that we recevied the data!
   # The b at the beginning is used to indicate bytes!
   with Detector(0, (640, 360)) as d:
      target = None
      while True:
         frame, target, center_x, center_y = d.detect(target)
         cv2.imshow("hello", frame)

         if center_x:
            center_x = int((center_x + 1)*100)
            center_y = int((center_y + 1)*100)
            
            if(center_y > 30):
               if(center_x < 90): #turn left
                  drive = bytearray()
                  drive.append(LR)
                  drive.append(100)
                  serialPort.write(drive)

                  drive = bytearray()
                  drive.append(RF)
                  drive.append(100)
                  serialPort.write(drive)
                  print(f"Center: ({center_x}, {center_y})") 


               elif(center_x > 110 ): # turn right
                  drive = bytearray()
                  drive.append(LF)
                  drive.append(100)
                  serialPort.write(drive)

                  drive = bytearray()
                  drive.append(RR)
                  drive.append(100)
                  serialPort.write(drive)
                  print(f"Center: ({center_x}, {center_y})") 

               else:
                  drive = bytearray()
                  drive.append(LF)
                  drive.append(100)
                  serialPort.write(drive)

                  drive = bytearray()
                  drive.append(RF)
                  drive.append(100)
                  serialPort.write(drive)
                  print(f"Center: ({center_x}, {center_y})") 
         else:
               
            drive = bytearray()
            drive.append(LR)
            drive.append(0)
            serialPort.write(drive)

            drive = bytearray()
            drive.append(RF)
            drive.append(0)
            serialPort.write(drive)
            print(f"Center: ({center_x}, {center_y})") 

