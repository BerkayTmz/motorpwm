from serial import Serial
import redis
from sys import argv

client = redis.Redis()
pubsub = client.pubsub()
pubsub.psubscribe("serial")

with Serial(argv[1], baudrate=350000) as serial:
    counter = 0
    while True:
        serial_image = client.get("serial_image") 
        if serial_image:
            serial.write(serial_image)
            serial.write('*'.encode('ascii'))
