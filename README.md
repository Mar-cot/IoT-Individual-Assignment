# IoT-Individual-Assignment 
Individual Assignment for "INTERNET-OF-THINGS ALGORITHMS AND SERVICES" by prof. Andrea Vitaletti  and prof. Ioannis Chatzigiannakis (course 10606829, Sapienza Università di Roma)



# Alumni Info

| Name | Surname | Matricola |
|---|---|---|
| Marco | Cotogni | 2046546 |

## Table of Contents
- [IoT-Individual-Assignment](#iot-individual-assignment)
- [Alumni Info](#alumni-info)
  - [Table of Contents](#table-of-contents)
- [Requirements](#requirements)
- [Architecture Overview](#architecture-overview)
  - [Hardware](#hardware)
  - [Pinout](#pinout)
  - [Software](#software)
- [The I2S Journey](#the-i2s-journey)
  - [What is I2S?](#what-is-i2s)
  - [Configuration](#configuration)
    - [Generator (TX) Setup](#generator-tx-setup)
    - [Sampler (RX) Setup](#sampler-rx-setup)
  - [Max Sample Rate](#max-sample-rate)
    - [**Why?**](#why)
    - [Is it safe to work at the limits?](#is-it-safe-to-work-at-the-limits)
    - [**Requirements satisfaction**](#requirements-satisfaction)
  - [Performing FFT](#performing-fft)
    - [Theroretical maximum](#theroretical-maximum)
    - [Parameters](#parameters)
    - [Memory Limitations](#memory-limitations)
    - [ESP-DSP API](#esp-dsp-api)
    - [The DSP Pipeline](#the-dsp-pipeline)
    - [Frequency Adaptation](#frequency-adaptation)
  - [I2S driver update](#i2s-driver-update)
    - [Switching to APLL (Audio Phase-Locked Loop)](#switching-to-apll-audio-phase-locked-loop)
  - [The Hybrid Pipeline](#the-hybrid-pipeline)
  - [Computing the aggregate value](#computing-the-aggregate-value)
- [Communication using MQTT](#communication-using-mqtt)
  - [MQTT Broker](#mqtt-broker)
  - [Communication Task](#communication-task)
- [Energy consumption](#energy-consumption)
  - [Energy saving techniques](#energy-saving-techniques)
  - [Monitoring Infrastructure](#monitoring-infrastructure)
  - [Oversampling vs. Adaptive Sampling](#oversampling-vs-adaptive-sampling)
    - [Oversampling](#oversampling)
    - [Adaptive Sampling](#adaptive-sampling)
    - [Comparison](#comparison)
- [Execution time and latency evaluation](#execution-time-and-latency-evaluation)
  - [Window execution time](#window-execution-time)
  - [Volume of Data Transmitted](#volume-of-data-transmitted)
  - [End-To-End Latency](#end-to-end-latency)
- [Bonus](#bonus)
  - [Signal 1 (50Hz) and Signal 2 (5kHz) Duty Cycle Analysis](#signal-1-50hz-and-signal-2-5khz-duty-cycle-analysis)
  - [Signal 3 (Lowest Amplitude Limit)](#signal-3-lowest-amplitude-limit)
- [LLM prompts](#llm-prompts)

---

# Requirements

1. Find out the maximum sampling rate of the system.
2. Generator must be able to produce an output signal containing frequencies greater or equal than half the maximum sampling rate of the system.
3. Generator must produce an output signal in the form of SUM(a_k*sin(f_k)).
4. Sampler must identify correctly the max frequency of the input signal.
5. Sampler must adapt its sample rate according to the Nyquist-Shannon Theorem.
6. Sampler must compute the average value of the signal over a time window
7. Sampler must communicate the average value of the signal using MQTT.
8. Evaluate the savings in energy of the new/adaptive sampling frequency against the original over-sampled one.
9. Measure per-window execution time.
10. Measure the volume of data transmitted over the network when the new/adaptive sampling frequency is used against the original over sampled one.
11. Measure the end-to-end latency of the system. From the point the data are generated up to the point they are received from the edge server.

**Personal goal**
- Learn how to use I2S and push it to its limits.

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

For the generator, I use the DAC provided in the MCU. In the case of ESP32 DOIT DEVKIT V1, the DAC sits on pins 25 (right channel) and 26 (left channel).

As for the sampler, I use the ADC provided in the MCU. In the case of the code contained in the repository, I chose to use pin 34 (Unit 1, Channel 6).

<img src="Final%20Figures/Architecture_overview.png" alt="Architecture Overview" width="75%">

## Software
The code was implemented using the arduino framework provided by the PlatformIO extension in VSCode.

I make full use of both the ESP32 Cores, handling multitasking with FreeRTOS:
- `Core 0` is entirely dedicated to sampling, as I didn't want to miss a sample from the generator.
- `Core 1` is dedicated to all the processing and communication tasks performed by the software.

# The I2S Journey
## What is I2S?
**Inter-IC Sound (I2S)** is a synchronous serial communication protocol traditionally used for connecting digital audio devices. In this project, it serves as the "high-speed highway" that bypasses standard CPU-bound sampling to interface directly with the ESP32's internal **ADC** and **DAC**. 

By leveraging **Direct Memory Access (DMA)**, the I2S peripheral can fill large buffers (e.g., `BUFFER_LEN = 64`) in the background. This allows the CPU to perform heavy digital signal processing without missing a single sample, which is critical for maintaining the signal integrity required for accurate frequency analysis.

<img src="Final%20Figures/standard_msb_diagram.png" alt="I2S Standard MSB Diagram" width="75%">


As illustrated in the timing diagrams, the protocol relies on three main signals:
* **BCLK (Bit Clock):** The timing pulse that dictates when data bits are read. The clock is constructed dividing the MCLK (Master Clock) frequency by an 8-bit unsigned number (_Spoiler_: it will give problems).
* **WS (Word Select):** Also known as LRCLK, it defines the start of a data frame and identifies the audio channel (Left/Right).
* **DIN/DOUT:** The actual payload of digital audio samples.


---

## Configuration
### Generator (TX) Setup
```c++
   i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false
  };
```

The generator uses `I2S_MODE_DAC_BUILT_IN` to drive the internal DACs on pins 25 and 26.

`.channel_format` tells the generator to post on both channels.
> **N.B:** As specified on the Pinout, the sampler will only read values written to the _Right Channel (Channel 1, PIN 25)_ 
>
> Then __why write on both DAC channels?__ The answer lies in a roadblock I stumbled upon: at first I tried setting _.channel_format_ to the standard `I2S_CHANNEL_FMT_ONLY_RIGHT`, and it led to the generator posting on the DMA twice as fast on the right channel.
>
> Indeed it worked as expected, ignoring the WS signal and just posting on the right channel, but the sample rate was being doubled aswell.
>
>The only fix I found was changing the flag to `I2S_CHANNEL_FMT_RIGHT_LEFT` and using some clever bit-shift tricks to effectively post 32-bit values on the DMA (16 bit for right channel and 16 bit for the left channel both containing the 16 bit sample).

$$\text{out\_buffer}[i] = ((\text{uint32\_t})\text{dac\_ready} \ll 16) \mid \text{dac\_ready}$$


### Sampler (RX) Setup
```c++
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = current_sample_rate, 
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,       
    .communication_format = I2S_COMM_FORMAT_I2S_MSB, 
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_LEN,
    .use_apll = false 
  };

  i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_6);
```

The sampler uses `I2S_MODE_ADC_BUILT_IN` to drive the internal ADC on pin 34 (Unit 1, Channel 6).
<!-- >**Coming up**: _why .use_apll = true_? -->


## Max Sample Rate
What is the __maximum sample rate__ I can achieve using this setup? (Req.1)

Using I2S I hit an API limit (in both DAC and ADC mode): __5MHz__.
### **Why?**
The BCLK is created from the MCLK (which in the case of ESP32 is 160MHz) via this formula:


$\text{SampleRate} \times 16 \text{ bits} \times 2 \text{ channel} = \text{ Hz BCLK}$

So, the absolute maximum BLCK the system can compute is 5MHz:

$5,000,000 \text{ Hz} \times 16 \text{ bits} \times 2 \text{ channel} = 160,000,000 \text{ Hz BCLK (160 MHz)}$

### Is it safe to work at the limits?
__Via empirical tests__, at 5MHz sample rate the communication becomes very sensible to noise so, for the rest of this project, __I will consider 100kHz as the reliable sampling rate limit__ (a.k.a the maximum sample rate that isn't subject to noises), and both DAC and ADC will be set to this sample rate.

### **Requirements satisfaction**
(Req.2) can be considered satisfied, since DAC has the same technical limits of the ADC.

As for (Req.3), I noticed that, when working with a sum of signals, the sin calculations became too slow to keep up with the high sampling rate (using _math.h_ library).

My solution was to implement a precalculation of the sinusoids (using _Wavetables_), which solved the issue but introduced a constraint on the frequencies available (es. 20Hz becomes 25.06Hz because 20Hz doesn't allow for a perfect cycle in the _Wavetable_, so to prevent a "jumping" signal I opted for a smart generator that snaps to the nearest "perfect cycle" frequency).  


## Performing FFT
To satisfy (Req.4) and (Req.5) the sampler must identify the maximum frequency contained in the signal, so how does it do that? It uses __Fast Fourier Transform__.

### Theroretical maximum

The theoretical maximum frequency the sampler can identify is ~**2.5MHz**.

But, as specified before, when working at the limits we get more sensible to noise, and computing the FFT yields mostly wrong results.   
Also, at high frequencies, the system is more subject to its memory and processing constraints.

### Parameters
To work around the constraints, we introduce these parameters: 

- `SAMPLES`:  Number of samples recorded to compute the FFT on. It must be a power of 2 (due to how FFT works), and the maximum it can be set to **4096**, which is what we use.
- `DECIMATION_FACTOR`: Controls how many of the samples the system keeps to perform the FFT, allowing the FFT to be computed on a larger time window. Empirically set to **2** (which effectively means that we keep 1 sample out of 2).

These parameters induce some properties of the FFT:
- ___New FFT Frequency___ = ${\text{SampleRate}\over\text{DecimationFactor}}$ Hz.
- ___What is the maximum frequency identifiable via FFT?___ $\text{FFTFrequency} \over 2$ Hz.
- ___How large is the time window of the FFT?___ $({1\over \text{FFTFrequency}}) \times \text{Samples}$ seconds.
- ___What is the resolutions of the FFT bins?___ ${\text{FFTFrequency}\over\text{Samples}}$ Hz.


### Memory Limitations

The ESP32 DOIT DEVKIT V1 comes with ~327KB of RAM divided into SRAM and DRAM.

Even though we should be able to use all of the RAM available, the ESP32 doesn't allow (at compile time) for a single allocation to be greater than ~180KiB (__dram0_0_seg fault).

<img src="Final%20Figures/FFT_Memory_Usage.png" alt="FFT Memory Overflow example" width="75%">

Using ***arduinoFFT*** library, I instantly hit the memory limit by just trying to allocate the buffers, so I switched to __esp_dsp__ API provided by ESP-IDF.


### ESP-DSP API
The *esp_dsp* API provides functions to calculate the FFT faster (using dedicated hardware named FPU (floating point unit)) and with less memory usage over _arduinoFFT_ (using in-place calculation in an interleaved array of real and imaginary numbers).

>The only real cons is that the API limits us to use 4096 samples (as the precalculated sin tables provided are of size 4096).

### The DSP Pipeline
* **Windowing:** To prevent spectral leakage, a **Hann Window** is pre-calculated and applied to the raw samples using `dsps_wind_hann_f32`.
* **Transformation:** The system executes a Radix-2 Complex FFT via `dsps_fft2r_fc32` on a buffer of $4096$ samples.
* **Reverse Search Algorithm:** After calculating the magnitude of each bin, the system establishes a noise threshold at **5%** of the maximum detected magnitude, and iterates backwards (from the highest to the lowest frequency bins), to find the first bin whose magnitude surpasses the threshold.

### Frequency Adaptation
Once the highest frequency bin is found ($f_{max}$), the system applies the Nyquist-Shannon Theorem (Req.5).

To ensure a safe margin for reconstruction, the adaptive sampling rate ($f_s$) is set as:
$$f_s = f_{max} \cdot 2.1$$

## I2S driver update
To update the I2S sample rate, we are forced to uninstall and re-install the I2S driver, using the dedicated API.

>___Question:___ What is the lowest sample rate we can set I2S to? Why?

On my first test I tried using a ~3.8kHz signal and $f_{max}$ correctly evaluated to ~8kHz, but when the system tried to set the sample rate of I2S, the API quietly set the sample rate to ~44kHz. 

__What happened?__ Recalling the I2S implementation, the BCLK is generated by dividing the MCLK using a register named `clkm_div`, which (sadly) it's an 8-bit unsigned register that tells the hardware the dividend of the MCLK.
This register overflowed (it wraps around after 255), and caused the system to set a wrong sample rate. What's worst is that the API thinks it's all ok, returning an `ESP_OK` value.

<img src="Final%20Figures/I2S_8-BIT-Overflow.png" alt="I2S 8-bit Register Overflow" width="75%">


__What's the limit?__ The lowest sample rate we can set, using the default MCLK, is ~**19.6kHz**

### Switching to APLL (Audio Phase-Locked Loop)

The I2S driver also allows use of a specialized clock, named APLL Clock (`.use_apll = true`), which is dinamically generated based on the I2S driver needs.

>I2S and APLL clock are meant for audio use, so the **ESP32 APLL Clock generator won't go below ~5.3MHz**.

Using APLL we unlocked a new mathematical floor of ~654Hz, even though the lowest _stable_ sample rate is empirically ~725Hz.

Since I wanted my system to be able to adapt to low frequencies, I wasn't satisfied with that. To solve the issue, I implementented an hybrid setup which uses I2S for high frequencies and standard ADC for low frequencies.

## The Hybrid Pipeline
The hybrid adaptive frequency setup follows these steps:
- Oversample the signal using I2S and perform FFT.

If $f_{max} > 4000$:
- Adapt the sample rate of the I2S driver to the Nyquist-Shannon frequency.

If $f_{max} \leq 4000$:
- Uninstall the I2S driver and use the standard ADC.
- Set sample rate to 5kHz and compute an higher resolution FFT (~1.2Hz per bin).
- Compute the new Nyquist-Shannon Frequency and adapt the sample rate (controlled by a delay in the ADC loop) to the new $f_{max}$.

<img src="Final%20Figures/Hybrid_pipeline1.png" alt="Hybrid Pipeline Diagram" width="75%">

**This hybrid setup allowed the system to adapt to the whole 1-25000Hz signal range.**


## Computing the aggregate value

After adapting its sample rate, the sampler computes the average value of the signal over a temporal window specified in the `AGGREGATE_WINDOW_SECONDS` parameter (Req.6).

At software level, the system keeps both the sum of the signal and the running count of how many samples it has seen, and when the sample count gets to the specified value, it computes the average and sends it to the communication task.

>The result is almost always **in 1.7k-1.8k range** which is what we expect from a 12-bit ADC (~4095/2).


# Communication using MQTT
To fulfill (Req.7), after the system computes the average, it uses MQTT over WiFi to communicate the result to an MQTT Broker.

## MQTT Broker
As a broker, I used an MQTT provider named [ThingsBoard Cloud](https://thingsboard.io/).

ThingsBoard is an open-source IoT platform for easy device management and data collection, and provides a dashboard to visualize the collected data.

<img src="Final%20Figures/ThingsBoard_Dashboard.png" alt="ThingsBoard_Dashboard" width="75%">

## Communication Task
Inside the sampler, using the ThingsBoard 0.15.0 library, `aggregateConsumerTask` waits for the result of the processing task pushed through the `aggregateQueue`. This `aggregateConsumerTask` acts as a telemetry gateway, ensuring data reaches the cloud while strictly managing the device's power profile.

To achieve high energy efficiency, the system avoids keeping the WiFi radio in an idle, power-consuming state. Instead, the task follows a strict lifecycle for every data window:

* **Radio Activation**: It triggers `connectNetwork()`, which initializes the WiFi station, connects to the SSID, and establishes a session with the ThingsBoard broker.
* **Data Transmission**: The average value is published using `tb.sendTelemetryData()`.
* **Immediate Teardown**: To eliminate the high power overhead of an idle radio, the system calls `disconnectNetwork()`. This function executes `WiFi.disconnect(true, true)` and sets the radio mode to `WIFI_OFF`, physically cutting power to the RF synthesizer.

>By using this "burst" communication strategy, the sampler remains in a low-power state for the majority of its operation, only consuming peak current during the brief intervals required to update the ThingsBoard dashboard.

# Energy consumption

## Energy saving techniques

The system achieves significant power reduction by minimizing the activity of the most power-hungry components: the CPU, the WiFi radio, and the high-speed I2S clock tree.

* **Hardware Light Sleep**: In the `analogReadTask`, the system calculates the time remaining until the next required sample. If this interval exceeds $400\mu s$ (a light sleep cycle takes ~$380\mu s$), it invokes `esp_light_sleep_start()`, dropping the current consumption of the ESP32 significantly until the next sample is due.
* **Critical Task Interlock**: To prevent system instability, a `critical_task_running` flag acts as an interlock. This prevents the MCU from entering sleep mode while the WiFi stack is initializing or transmitting data, ensuring that energy-saving measures do not interfere with reliable communication.
* **Radio Power Management**: The WiFi radio is kept in `WIFI_OFF` mode for the vast majority of the time. It is only powered on during the brief window required to transmit the telemetry data to ThingsBoard, after which it is immediately disconnected and shut down to cut power to the RF synthesizer.
* **Peripheral Gating**: When the detected signal frequency is low ($< 4000$ Hz), the system switches from the I2S-DMA pipeline to a manual ADC loop. During this transition, `i2s_driver_uninstall()` is called to completely disable the I2S hardware clock tree, which would otherwise consume power even when idle.

## Monitoring Infrastructure
To obtain precise, non-intrusive power metrics without affecting the performance of the Sampler or Generator, a dedicated monitoring subsystem was established:

* **INA219 Current Sensor**: Used for high-side DC current and voltage monitoring of the Sampler ESP32's 5V rail.
* **Dedicated Data Logger**: A third ESP32 serves as an isolated monitor, polling the INA219 via I2C to log real-time consumption data without adding overhead to the experimental units.
* **External Power Supply**: A stable 5V external source was used to ensure measurements were not skewed by USB voltage fluctuations.

<img src="Final Figures/INA_pinout.png" alt="INA219 Pinout" width="75%">

## Oversampling vs. Adaptive Sampling
For (Req.8) I measured the energy consumption of the sampler, without considering communication.
Both Oversampling and Adaptive sampling were evaluated considering a 20Hz signal.

### Oversampling
During Oversampling, the sampler uses I2S driver at max `SAMPLE_RATE` (100kHz).

<img src="Final%20Figures/Oversampling_first50s.png" alt="Oversampling energy consumption" width="100%">

>After boot, we get an energy consumption of about **~49mA**.

### Adaptive Sampling 
During Adaptive Sampling, the sampler first uses I2S driver at max `SAMPLE_RATE` to perform the FFT, but then switches to manual ADC loop and employes the light sleep technique to save energy.

<img src="Final%20Figures/Adaptive_sample_rate_first50s.png" alt="Adaptive sampling energy consumption" width="100%">

Looking at the graph we can distinguish between the first FFT I2S sweep (From 3s to 5s), then the second FFT ADC sweep (from 5s to 27s), then the switch to manual ADC loop and light sleep in-between samples.

>This is what 20Hz Sampling looks like in terms of energy consumption:

<img src="Final%20Figures/20Hz_sampling.png" alt="20Hz Sampling" width="100%">

### Comparison

Using a script, I analyzed the Oversampling vs. Adaptive Sample consumption and these are the results:
```
=== ADAPTIVE OFF ===
Duration (s):      39.999
Avg current (mA):  49.113
Avg power (mW):    247.798
Energy (mJ):       9911.714

=== ADAPTIVE ON ===
Duration (s):      39.998
Avg current (mA):  5.822
Avg power (mW):    29.562
Energy (mJ):       1182.429

=== SAVINGS (ON vs OFF) ===
Current saving (%): 88.14
Power saving (%):   88.07
Energy saving (%):  88.07
```
# Execution time and latency evaluation
## Window execution time
Since the windows are temporally defined by `AGGREGATE_WINDOW_SECONDS` (in the case of my tests I used 5), it takes **5s** to fully compute a window (Req.9). This implies basically zero CPU overhead.

<img src="Final%20Figures/Window_execution_time.png" alt="20Hz" width="100%">

## Volume of Data Transmitted
Inside my communication task I use the ThingsBoard library which automatically handles the JSON payload construction and publishing, using the `sendTelemetryData()` method.

The system sends a packet like `{"average_value": 1816.91}` with QoS 0. **Size: 26 bytes**.

ThingsBoard adds this necessary headers for the protocol:
* **Fixed Header (2 byte)**: Specifies packet type(PUBLISH) and the remaining lenght.
* **Topic Name (46 byte)**: ThingsBoard uses v1/devices/access_token/telemetry as a topic name. These are 44 characters plus 2 bytes to specify the lenght of the string.

>To sum up, for each transmission we transmit **74 bytes** of data (Req.10).

## End-To-End Latency
Using the _Radio Power Management_ technique of turning the radio on only when needed, introduces a lot of latency in the system.

The `aggregateConsumerTask` (my communication task), also accounts for keeping track of the latency of the system.


For every packet sent, it prints:
```
>>> [THINGSBOARD PUBLISHED] 
- average_value: 1816.91
- delta_time: 2793855 us
- packet_id: 1
- packet_rtt: 3889 us
- sleep_count: 808
```
- `average_value`: the average value of the signal in the temporal window.
- `delta_time`: how much time it takes for the system to turn ON the radio, connect to the WiFi, send a packet and turn OFF the radio.
- `packet_id`: id of packet sent.
- `packet_rtt`: Round Trip Time of the packet.
- `sleep_count`: How many light sleep cycles were performed since last packet sent.

Since turning off the radio and the packet rtt takes negligble time, we can say that the end-to-end latency of the is **~2.8s** (Req. 10), but heavily depends on how much time it takes for the ESP32 to connect to the WiFi.


# Bonus

## Signal 1 (50Hz) and Signal 2 (5kHz) Duty Cycle Analysis
For the bonus analysis I decided to analyze the duty cycles of 2 signals using the adaptive sampling frequency configuration, over a 10s aggregation window. 

Signal 1:
$sin(2\pi \dot{50}t)$

>50Hz Duty Cycle
<img src="Final%20Figures/Duty_Cycle_50Hz_10s_window.png" alt="50Hz Duty Cycle" width="100%">

We can easily distinguish between the sampling phase and the communication phase. The average energy consumption is **~47.3mA**

Signal 2:
$sin(2\pi \dot{5000}t)$

>5kHz Duty Cycle
<img src="Final%20Figures/Duty_Cycle_5kHz_10s_window.png" alt="5kHz Duty Cycle" width="100%">



Even with 5kHz sampling we can easily distinguish between sampling and communicating, but the communication spike is much lower.

This happens because for 5kHz signals the system keeps the I2S driver on, which draws more energy.
Indeed, the average energy consumption is **~72.5mA** 

## Signal 3 (Lowest Amplitude Limit)
For the third signal I experimented with the sensitivity of the FFT.

**What is the minimum amplitude required for a pure sinusoid to be identified as a peak rather than being filtered out as noise?**

I experimented with this signal: 
$sin(2\pi \dot{25}t)+sin(2\pi \dot{200}t)+x sin(2\pi \dot{1000}t)$

Empirically, I found these thresholds:
* $x \ge 0.07$: FFT is able to correctly identify the signal.
* $0.05\le x \lt 0.07$: FFT fails during the I2S Sweep phase, but finds the 1kHz frequency when ADC Sweeping.
* $x \lt 0.05$: FFT does not identify 1kHz as the max frequency.


# LLM prompts
For this project I used Gemini 3.1 Pro.

I tried to kept everything in one single chat, prompting the LLM to refresh his memory and attaching the code to modify.
It didn't always work but I noticed improvements over not prompting to refresh, and when prompted it took in consideration the manual changes in-between prompts.

[Chat](https://gemini.google.com/share/72eb2c112180)
 