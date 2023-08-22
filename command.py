import serial
import sys

with serial.Serial('/dev/ttyUSB1', baudrate=115200) as s:
    s.write(sys.argv[1].encode('utf-8'))
