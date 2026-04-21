import numpy as np

# =========================
# CONFIG
# =========================
file_off = "adaptive_off.txt"
file_on = "adaptive_on.txt"

TAIL_SECONDS = 40.0  # usa solo gli ultimi 40 secondi

# =========================
# LOAD FUNCTION
# =========================
def load_run(filename):
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
            except ValueError:
                continue

            time_ms.append(t)
            current_mA.append(c)
            power_mW.append(p)

    if len(time_ms) < 2:
        raise ValueError(f"Dati insufficienti in {filename}")

    time_s = np.array(time_ms) / 1000.0
    current_mA = np.array(current_mA)
    power_mW = np.array(power_mW)

    # Rendi tempo relativo
    time_s = time_s - time_s[0]

    # ===== PRENDI SOLO GLI ULTIMI 40s =====
    t_end = time_s[-1]
    t_start = t_end - TAIL_SECONDS

    mask = time_s >= t_start

    time_s = time_s[mask]
    current_mA = current_mA[mask]
    power_mW = power_mW[mask]

    # Riporta tempo a partire da 0
    time_s = time_s - time_s[0]

    if len(time_s) < 2:
        raise ValueError(f"Non abbastanza dati negli ultimi {TAIL_SECONDS}s in {filename}")

    return time_s, current_mA, power_mW

# =========================
# METRICS
# =========================
def compute_metrics(filename):
    time_s, current_mA, power_mW = load_run(filename)

    duration_s = time_s[-1] - time_s[0]

    avg_current_mA = np.mean(current_mA)
    avg_power_mW = np.mean(power_mW)

    # Energia = integrale potenza nel tempo (mJ)
    energy_mJ = np.trapz(power_mW, time_s)
    energy_mWh = energy_mJ / 3600.0

    return {
        "filename": filename,
        "duration_s": duration_s,
        "avg_current_mA": avg_current_mA,
        "avg_power_mW": avg_power_mW,
        "energy_mJ": energy_mJ,
        "energy_mWh": energy_mWh,
    }

def savings_percent(ref, new):
    return 100.0 * (ref - new) / ref

# =========================
# MAIN
# =========================
off = compute_metrics(file_off)
on = compute_metrics(file_on)

print("=== USING LAST 40 SECONDS ONLY ===\n")

print("=== ADAPTIVE OFF ===")
print(f"Duration (s):      {off['duration_s']:.3f}")
print(f"Avg current (mA):  {off['avg_current_mA']:.3f}")
print(f"Avg power (mW):    {off['avg_power_mW']:.3f}")
print(f"Energy (mJ):       {off['energy_mJ']:.3f}")
print()

print("=== ADAPTIVE ON ===")
print(f"Duration (s):      {on['duration_s']:.3f}")
print(f"Avg current (mA):  {on['avg_current_mA']:.3f}")
print(f"Avg power (mW):    {on['avg_power_mW']:.3f}")
print(f"Energy (mJ):       {on['energy_mJ']:.3f}")
print()

print("=== SAVINGS (ON vs OFF) ===")
print(f"Current saving (%): {savings_percent(off['avg_current_mA'], on['avg_current_mA']):.2f}")
print(f"Power saving (%):   {savings_percent(off['avg_power_mW'], on['avg_power_mW']):.2f}")
print(f"Energy saving (%):  {savings_percent(off['energy_mJ'], on['energy_mJ']):.2f}")
print()

if savings_percent(off["energy_mJ"], on["energy_mJ"]) > 0:
    print("Result: adaptive ON consumed LESS")
else:
    print("Result: adaptive ON did NOT reduce consumption")