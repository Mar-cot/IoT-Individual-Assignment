# IoT-Individual-Assignment
Individual Assignment for "INTERNET-OF-THINGS ALGORITHMS AND SERVICES" by prof. Andrea Vitaletti  and prof. Ioannis Chatzigiannakis (course 10606829, Sapienza Università di Roma)

### Alumni Info

| Name | Surname | Matricola |
|---|---|---|
| Marco | Cotogni | 2046546 |


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

## Personal goal
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

For the generator, we use the DAC provided in the MCU. In the case of ESP32 DOIT DEVKIT V1, the DAC sits on pins 25 (right channel) and 26 (left channel).

As for the sampler, we use the ADC provided in the MCU. In the case of the code contained in the repository, I chose to use pin 34 (Unit 1, Channel 6).

<img src="Final%20Figures/Architecture_overview.png" alt="Architecture Overview" width="75%">


# The I2S Journey
## What is I2S?
**Inter-IC Sound (I2S)** is a synchronous serial communication protocol traditionally used for connecting digital audio devices. In this project, it serves as the "high-speed highway" that bypasses standard CPU-bound sampling to interface directly with the ESP32's internal **ADC** and **DAC**. 

<img src="Final%20Figures/standard_msb_diagram.png" alt="I2S Standard MSB Diagram" width="75%">


As illustrated in the timing diagrams, the protocol relies on three main signals:
* **BCLK (Bit Clock):** The timing pulse that dictates when data bits are read. The clock is constructed dividing the MCLK (Master Clock) frequency by an 8-bit unsigned number (_Spoiler_: it will give problems).
* **WS (Word Select):** Also known as LRCLK, it defines the start of a data frame and identifies the audio channel (Left/Right).
* **DIN/DOUT:** The actual payload of digital audio samples.

By leveraging **Direct Memory Access (DMA)**, the I2S peripheral can fill large buffers (e.g., `BUFFER_LEN = 64`) in the background. This allows the CPU to perform heavy digital signal processing without missing a single sample, which is critical for maintaining the signal integrity required for accurate frequency analysis.

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
> **N.B:** As specified on the Pinout, we only read values written to the _Right Channel (Channel 1, PIN 25)_ 
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
What is the __maximum sample rate__ we can achieve using this setup? (Req.1)

Using I2S we hit an API limit (in both DAC and ADC mode): __5MHz__.
#### Why?
The BCLK is created from the MCLK (which in the case of ESP32 is 160MHz) via this formula:


$\text{SampleRate} \times 16 \text{ bits} \times 2 \text{ channel} = \text{ Hz BCLK}$

So, the absolute limit we can hit is 5MHz:

$5,000,000 \text{ Hz} \times 16 \text{ bits} \times 2 \text{ channel} = 160,000,000 \text{ Hz BCLK (160 MHz)}$

#### Is it useful?
__Via empirical tests__, at 5MHz sample rate the communication becomes very noisy so, for the rest of this project, __I will consider 100kHz as the reliable sampling rate limit__, and both DAC and ADC will be set to this sample rate.

#### Requirements satisfaction
(Req.2) can be considered satisfied, since DAC has the same technical limits of the ADC.

As for (Req.3), I noticed that for multiple signals the sin calculations became too slow to keep up with the high sampling rate (using _math.h_ library).

My solution was to implement a precalculation of the signals (using _Wavetables_), which solved the issue but introduced a constraint on the frequencies available (es. 20Hz becomes 25.06Hz because 20Hz doesn't allow for a perfect cycle in the _Wavetable_, so to prevent a "jumping" signal I opted for a smart generator that snaps to the nearest "perfect cycle" frequency).  

## Performing FFT


### The DSP Pipeline
* **Windowing:** To prevent spectral leakage, a **Hann Window** is pre-calculated and applied to the raw samples using `dsps_wind_hann_f32`.
* **Transformation:** The system executes a Radix-2 Complex FFT via `dsps_fft2r_fc32` on a buffer of $4096$ samples.
* **Magnitude and Noise Floor:** After calculating the magnitude of each bin, the system establishes a noise threshold at **5%** of the maximum detected magnitude to ignore background artifacts.

### Frequency Adaptation
Once the highest frequency component ($f_{max}$) above the threshold is found, the system applies the Nyquist-Shannon Theorem. To ensure a safe margin for reconstruction, the adaptive sampling rate ($f_s$) is set as:
$$f_s = f_{max} \cdot 2.1$$
This optimization ensures that the system only captures the necessary amount of data, directly leading to reduced execution time and lower network telemetry volume.


# Communication using MQTT

# Energy consumption evaluation

# Execution time and latency evaluation

# Bonus

# LLM prompts
For this project I used Gemini 3.1 Pro.

I tried to kept everything in one single chat, prompting the LLM to refresh his memory and attaching the code to modify.
It didn't always work but I noticed improvements over not prompting to refresh, and when prompted it took in consideration the manual changes in-between prompts.

[Chat](https://gemini.google.com/share/72eb2c112180)
 