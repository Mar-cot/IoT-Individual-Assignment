import serial
import time


ser = serial.Serial('COM3', 921600)  
filename = "adaptive_on_no_aggregation_50Hz_10s_window.txt"  

start_time = time.time()

with open(filename, "w") as f:
    while time.time() - start_time < 180:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print(line)
            f.write(line + "\n")

ser.close()