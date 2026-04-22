import numpy as np
import matplotlib.pyplot as plt

# ====== CONFIG ======
filename = "adaptive_off_pure_sampling.txt" 
title_prefix = "Oversampling with 20Hz Signal"

# ====== LOAD DATA ======
time_ms = []
current_mA = []
power_mW = []

with open(filename, "r") as f:
    for line in f:
        parts = line.strip().split()
        
        if len(parts) != 3:
            continue
        
        try:
            t = float(parts[0])
            c = float(parts[1])
            p = float(parts[2])
        except:
            continue
        
        time_ms.append(t)
        current_mA.append(c)
        power_mW.append(p)

time_ms = np.array(time_ms)
current_mA = np.array(current_mA)
power_mW = np.array(power_mW)

time_s = time_ms / 1000.0

# ====== (OPZIONALE) SMOOTHING ======
window = 10
power_mW_smooth = np.convolve(power_mW, np.ones(window)/window, mode='same')
current_mA_smooth = np.convolve(current_mA, np.ones(window)/window, mode='same')

# ====== PLOT POWER ======
plt.figure(figsize=(10, 5))
plt.plot(time_s, power_mW_smooth)
plt.xlabel("Time (s)")
plt.ylabel("Power (mW)")
plt.title(f"{title_prefix} - Power")
plt.grid(True)
plt.savefig(filename.replace(".txt", "_power.png"), dpi=300)
plt.show()

# ====== PLOT CURRENT ======
plt.figure(figsize=(10, 5))
plt.plot(time_s, current_mA_smooth)
plt.xlabel("Time (s)")
plt.ylabel("Current (mA)")
plt.title(f"{title_prefix} - Current")
plt.grid(True)
plt.savefig(filename.replace(".txt", "_current.png"), dpi=300)
plt.show()