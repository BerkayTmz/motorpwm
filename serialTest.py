import serial
import time
serialPort = serial.Serial(port = "COM0", baudrate=115200,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

serialString = ""                           # Used to hold data coming over UART

while(1):
   # Wait until there is data waiting in the serial buffer
   # Print the contents of the serial data
   # Tell the device connected over the serial port that we recevied the data!
   # The b at the beginning is used to indicate bytes!
   x = 0x00
   serialPort.write(x)
   x = x+1
   time.sleep(100)   
