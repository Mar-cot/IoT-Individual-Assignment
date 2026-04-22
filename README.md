# IoT-Individual-Assignment
Individual Assignment for "INTERNET-OF-THINGS ALGORITHMS AND SERVICES" by prof. Andrea Vitaletti  and prof. Ioannis Chatzigiannakis (course 10606829, Sapienza Università di Roma)

### Alumni Info

| Name | Surname | Matricola |
|---|---|---|
| Marco | Cotogni | 2046546 |


# Requirements

- Find out the maximum sampling rate of the system.
- Generator must produce an output signal in the form of SUM(a_k*sin(f_k)).
- Generator must be able to produce an output signal containing frequencies greater or equal than half the maximum sampling rate of the system.
- Sampler must identify correctly the max frequency of the input signal.
- Sampler must adapt its sample rate according to the Nyquist-Shannon Theorem.
- Sampler must compute the average value of the signal over a time window.
- Sampler must communicate the average value of the signal using MQTT.
- Evaluate the savings in energy of the new/adaptive sampling frequency against the original over-sampled one.
- Measure per-window execution time.
- Measure the volume of data transmitted over the network when the new/adaptive sampling frequency is used against the original over sampled one.
- Measure the end-to-end latency of the system. From the point the data are generated up to the point they are received from the edge server.

## Personal goal
- Learn to use I2S and push it to its limits.

# Architecture Overview
## Hardware

**Strictly needed**
- 2 ESP32 DOIT DEVKIT V1 (One as a generator, the other as a sampler).
- 2 Jumper wires.
  
**Extra components to measure power consumption**
- 1 MCU that supports I2C (I used another ESP32).
- INA219 Module.
- External 5V power supply.
- More wires.

## Pinout
**Essential pinout**

For the generator, we use the DAC provided in the MCU. In the case of ESP32 DOIT DEVKIT V1, the DAC sits on pins 25 (right channel) and 26 (left channel).

As for the sampler, we use the ADC provided in the MCU. In the case of the code contained in the repository, I chose to use pin 34 (Unit 1, Channel 6).

<img src="Final%20Figures/Architecture_overview.png" alt="Architecture Overview" width="75%">


# The I2S Journey
## What is I2S?
## Configuration
## Performing FFT

# Communication using MQTT

# Energy consumption evaluation

# Execution time and latency evaluation

# Bonus

 