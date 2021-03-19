from serial import Serial
import redis

client = redis.Redis()
pubsub = client.pubsub()
pubsub.psubscribe("serial")

with Serial("/dev/ttyUSB1", baudrate=350000) as serial:
    counter = 0
    while True:
        serial_image = client.get("serial_image") 
        if serial_image:
            serial.write(serial_image)
            serial.write('*'.encode('ascii'))
