#include "driver/i2s.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_dsp.h" // Espressif's official hardware-accelerated DSP library

#define I2S_SAMPLE_RATE 100000
#define DECIMATION_FACTOR 2
#define FFT_SAMPLE_RATE (I2S_SAMPLE_RATE/DECIMATION_FACTOR) // ~50000 Hz
#define SAMPLES 4096
#define BUFFER_LEN 64

// FreeRTOS Handles
QueueHandle_t sampleQueue;
TaskHandle_t Task1Handle;
TaskHandle_t FFTTaskHandle;

// ESP-DSP Arrays
// It uses an interleaved format: [Real0, Imag0, Real1, Imag1...]
// So for 2048 samples, the array size is 4096.
float fft_data[SAMPLES * 2]; 
float window_coefficients[SAMPLES];

// ---------------------------------------------------------
// TASK 1: Read I2S (Pinned to Core 0)
// ---------------------------------------------------------
void i2sReadTask(void *pvParameters) {
  uint16_t buffer[BUFFER_LEN];
  size_t bytes_read;

  for (;;) {
    i2s_read(I2S_NUM_0, buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
    xQueueOverwrite(sampleQueue, buffer);
  }
}

// ---------------------------------------------------------
// TASK 2: Compute FFT (Pinned to Core 1)
// ---------------------------------------------------------
void fftTask(void *pvParameters) {
  uint16_t rx_buffer[BUFFER_LEN];
  int sample_index = 0;
  int indicesToKeep[BUFFER_LEN/DECIMATION_FACTOR];

  for(int i = 0; i < BUFFER_LEN/DECIMATION_FACTOR; i++){
    indicesToKeep[i] = i * DECIMATION_FACTOR;
    Serial.println(indicesToKeep[i]);
  }

  for (;;) {
    if (xQueueReceive(sampleQueue, rx_buffer, portMAX_DELAY) == pdPASS) {
      
      // Decimation: Keep 2 evenly spaced samples out of 64
      
      for (int i = 0; i < BUFFER_LEN/DECIMATION_FACTOR; i++) {
        if (sample_index < SAMPLES) {
          int buffer_idx = indicesToKeep[i];
          
          // ESP-DSP Interleaved Format
          // Even index = Real Part, Odd index = Imaginary Part
          fft_data[sample_index * 2] = (float)(rx_buffer[buffer_idx] & 0xFFF) - 2048.0; 
          fft_data[sample_index * 2 + 1] = 0.0; // Imaginary is 0
          
          sample_index++;
        }
      }

      // Once we have enough samples, run the hardware-accelerated FFT
      if (sample_index >= SAMPLES) {
        
        // 1. Apply Window (Multiply the real parts by the pre-calculated window)
        for (int i = 0; i < SAMPLES; i++) {
          fft_data[i * 2] *= window_coefficients[i];
        }

        // 2. Compute FFT (Extremely fast, uses ESP32 FPU hardware)
        dsps_fft2r_fc32(fft_data, SAMPLES);

        // 3. Bit-reverse the array (A required step in the ESP-DSP algorithm)
        dsps_bit_rev_fc32(fft_data, SAMPLES);

        // 4. Find the major peak manually (ESP-DSP leaves the array as complex numbers)
        float max_magnitude = 0;
        int peak_index = 0;

        // We only scan the first half of the array (up to the Nyquist limit)
        for (int i = 0; i < SAMPLES / 2; i++) {
          float real = fft_data[i * 2];
          float imag = fft_data[i * 2 + 1];
          
          // Magnitude = sqrt(real^2 + imag^2)
          float magnitude = sqrt((real * real) + (imag * imag));

          if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            peak_index = i;
          }
        }

        // 5. Convert the peak index back into a Frequency (Hz)
        float majorPeak = (float)peak_index * (FFT_SAMPLE_RATE / (float)SAMPLES);
        
        Serial.print("Dominant Frequency: ");
        Serial.print(majorPeak);
        Serial.println(" Hz");

        sample_index = 0; 
      }
    }
  }
}

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Initialize ESP-DSP for a maximum FFT size of 4096
  dsps_fft2r_init_fc32(NULL, 4096);
  
  // Generate Hann window coefficients once to save CPU time later
  dsps_wind_hann_f32(window_coefficients, SAMPLES);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = (uint32_t)I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,       
    .communication_format = I2S_COMM_FORMAT_STAND_I2S, 
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = true 
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_6);
  i2s_adc_enable(I2S_NUM_0);

  sampleQueue = xQueueCreate(1, sizeof(uint16_t) * 64);

  xTaskCreatePinnedToCore(i2sReadTask, "I2S_Read", 4096, NULL, 2, &Task1Handle, 0);
  xTaskCreatePinnedToCore(fftTask, "FFT_Process", 8192, NULL, 3, &FFTTaskHandle, 1);
}

void loop() {
  vTaskDelete(NULL); 
}