import serial
import time


ser = serial.Serial('COM4', 921600)  
filename = "adaptive_on_pure_sampling_20Hz.txt"  

start_time = time.time()

with open(filename, "w") as f:
    while time.time() - start_time < 180:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print(line)
            f.write(line + "\n")

ser.close()