from serial import Serial
import redis
from serial.tools import list_ports
from typing import Dict, Tuple, List

ESP32_VID_PID = [(6790, 29987)]


def get_device_com(comports, vid_pid_tuple: List[Tuple[int, int]]):
    device_com = [com.device for com in comports
                  if (com.vid, com.pid) in vid_pid_tuple]
    return device_com[0] if len(device_com) else None


def main():
    global ESP32_VID_PID
    client = redis.Redis()

    comports = list(list_ports.comports())
    esp32_dev_com = get_device_com(comports, ESP32_VID_PID)

    with Serial(esp32_dev_com, baudrate=350000) as serial:
        while True:
            serial_image = client.get('serial_image')
            if serial_image:
                serial.write(serial_image)
                serial.write('*'.encode('ascii'))


if __name__ == "__main__":
    main()

