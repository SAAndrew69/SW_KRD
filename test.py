#!/usr/bin/env python3

#!/usr/bin/env python3

import serial
import time
import random

ser = serial.Serial('COM4', 115200,timeout=0.01)  # open serial port
print(ser.name)         # check which port was really used

while True:
    time.sleep(0.05)
    r = hex(random.randint(0, 0xFFFF)) + '\n'
    ser.write(bytes(r, 'latin-1'))     # write a string
    print(r, end='')

ser.close()             # close port