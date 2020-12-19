import serial
from detector import Detector
import cv2

serialPort = serial.Serial(port = "COM0", baudrate=115200,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

serialString = ""                           # Used to hold data coming over UART

while(1):
   # Wait until there is data waiting in the serial buffer
   # Print the contents of the serial data
   # Tell the device connected over the serial port that we recevied the data!
   # The b at the beginning is used to indicate bytes!
   serialPort.write(b"Selamunaleykum haci abi\r\n")   
   with Detector(0, (640, 360)) as d:
      target = None
      while True:
         frame, target, center = d.detect(target)
         cv2.imshow("hello", frame)
         if center:
            print("Center")