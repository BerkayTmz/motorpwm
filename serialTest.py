import serial
import time
serialPort = serial.Serial(port = "/dev/ttyUSB0", baudrate=115200,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

serialString = ""                           # Used to hold data coming over UART
x = 0x00
RF = 10
RR = 15
LF = 20
LR = 25
while(1):
   # Wait until there is data waiting in the serial buffer
   # Print the contents of the serial data
   # Tell the device connected over the serial port that we recevied the data!
   # The b at the beginning is used to indicate bytes!
    drive = bytearray()
    drive.append(RF)
    drive.append(200)
    serialPort.write(drive)  
    print("Center: RF") 

    time.sleep(5)
    drive = bytearray()
    drive.append(RF)
    drive.append(0)
    serialPort.write(drive)  
    print("Center: RR") 

    drive = bytearray()
    drive.append(RR)
    drive.append(200)
    serialPort.write(drive)  
    print("Center: RR") 

    time.sleep(5)
    drive = bytearray()
    drive.append(RR)
    drive.append(0)
    serialPort.write(drive)  
    print("Center: RR") 
    drive = bytearray()
    drive.append(LF)
    drive.append(200)
    serialPort.write(drive)  
    print("Center: LF") 


    time.sleep(5)
    drive = bytearray()
    drive.append(LF)
    drive.append(0)
    serialPort.write(drive)  
    print("Center: LF") 
    drive = bytearray()
    drive.append(LR)
    drive.append(200)
    serialPort.write(drive)  
    print("Center: LR") 

    time.sleep(5)

