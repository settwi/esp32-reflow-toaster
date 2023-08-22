import serial
import time

with serial.Serial('/dev/ttyUSB1', baudrate=115200) as s:
    s.timeout = 2

    temp_mask = 0xfff
    gain = 0.25

    while True:
        a, b = s.read(2)
        comb = (a << 8) | b
        masked = (comb >> 3) & temp_mask
        temp = masked * gain
        print(f'temperature is: {temp:.2f}')
        time.sleep(0.1)
