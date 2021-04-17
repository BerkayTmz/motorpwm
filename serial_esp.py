from serial import Serial
import redis
from serial.tools import list_ports
from typing import Dict, Tuple

client = redis.Redis()
pubsub = client.pubsub()
pubsub.psubscribe("serial")

def get_device_com(comports, vid_pid_tuple: Tuple[int, int]):
    device_com = [com.device for com in comports
                  if (com.vid, com.pid) == vid_pid_tuple]
    return device_com[0] if len(device_com) else None
    
ESP32_VID_PID = (6790, 29987)
comports = list(list_ports.comports())
esp32_dev_com = get_device_com(comports, ESP32_VID_PID)

with Serial(esp32_dev_com, baudrate=350000) as serial:
    counter = 0
    while True:
        serial_image = client.get("serial_image") 
        if serial_image:
            serial.write(serial_image)
            serial.write('*'.encode('ascii'))
