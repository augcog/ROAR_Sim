import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1, writeTimeout=1)

serial_msg ='& {} {}\n'.format(1600, 1500)
print(serial_msg)
for i in range(10):
    print(i)
    ser.write(serial_msg.encode('ascii'))
    print(ser.read(1000))
# input("Input something")

time.sleep(5)
print("Time to stop")

ser.write('& {} {}\n'.format(1500, 1500).encode('ascii'))
