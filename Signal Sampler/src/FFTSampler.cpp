#include "driver/i2s.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_dsp.h" 

#define DECIMATION_FACTOR 2
#define SAMPLES 4096
#define BUFFER_LEN 64
#define MAX_FFT_RUNS 50 // The number of calibration runs

uint32_t current_i2s_rate = 100000; 
float current_fft_rate = (float)current_i2s_rate / DECIMATION_FACTOR; 

QueueHandle_t fftsampleQueue;
TaskHandle_t Task1Handle;
TaskHandle_t FFTTaskHandle;

// 1. DYNAMIC POINTERS: These replace the static arrays so we can free them later.
float *fft_data; 
float *window_coefficients;

// 2. STATE FLAG: Tells the I2S reader when to stop feeding the FFT.
volatile bool calibration_done = false;

// --- I2S READ TASK ---
void i2sReadTask(void *pvParameters) {
  uint16_t buffer[BUFFER_LEN];
  size_t bytes_read;

  for (;;) {
    i2s_read(I2S_NUM_0, buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
    
    if (!calibration_done) {
      // We are in calibration mode, feed the FFT task
      xQueueOverwrite(fftsampleQueue, buffer);
    } else {
      // ------------------------------------------------------------------
      // CALIBRATION IS OVER, THE HARDWARE IS OPTIMIZED.
      // This is where your final application goes! 
      // (e.g., save the buffer to an SD Card, send it over WiFi, etc.)
      // ------------------------------------------------------------------
      
      // Example: just to prove it's still running at the new optimized speed
      // Serial.println(buffer[0]); 
    }
  }
}

// --- FFT TASK ---
void fftTask(void *pvParameters) {
  uint16_t rx_buffer[BUFFER_LEN];
  int sample_index = 0;
  
  // Variables for the 50-run calibration
  int fft_run_count = 0;
  int recorded_peaks[MAX_FFT_RUNS]; 

  int indicesToKeep[BUFFER_LEN/DECIMATION_FACTOR];
  for(int i = 0; i < BUFFER_LEN/DECIMATION_FACTOR; i++){
    indicesToKeep[i] = i * DECIMATION_FACTOR;
  }

  for (;;) {
    if (xQueueReceive(fftsampleQueue, rx_buffer, portMAX_DELAY) == pdPASS) {
      
      for (int i = 0; i < BUFFER_LEN/DECIMATION_FACTOR; i++) {
        if (sample_index < SAMPLES) {
          int buffer_idx = indicesToKeep[i];
          fft_data[sample_index * 2] = (float)(rx_buffer[buffer_idx] & 0xFFF) - 2048.0; 
          fft_data[sample_index * 2 + 1] = 0.0; 
          sample_index++;
        }
      }

      if (sample_index >= SAMPLES) {
        
        for (int i = 0; i < SAMPLES; i++) {
          fft_data[i * 2] *= window_coefficients[i];
        }

        dsps_fft2r_fc32(fft_data, SAMPLES);
        dsps_bit_rev_fc32(fft_data, SAMPLES);

        // --- REVERSE ALGORITHM ---
        float max_magnitude = 0;
        
        // Skip DC offset (i = 0) to prevent false positives
        for (int i = 1; i < SAMPLES / 2; i++) {
          float real = fft_data[i * 2];
          float imag = fft_data[i * 2 + 1];
          
          // Calculate Linear Magnitude (not Energy)
          float magnitude = sqrt((real * real) + (imag * imag));
          
          // RAM OPTIMIZATION: Store magnitude back into the array
          fft_data[i] = magnitude; 
          
          // Keep track of the absolute loudest frequency to establish our noise floor
          if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
          }
        }

        // 5. Set the "First Strike" Threshold
        // We consider any signal that is at least 5% the volume of the loudest peak to be "Active"
        float noise_threshold = max_magnitude * 0.05f; 
        int highest_freq_index = 0;

        // 6. Top-Down Reverse Search
        // Start from the absolute highest frequency (Nyquist limit) and scan backwards
        for (int i = (SAMPLES / 2) - 1; i >= 1; i--) {
          
          if (fft_data[i] >= noise_threshold) {
            highest_freq_index = i;
            break; // We hit a real signal! Stop searching immediately.
          }
        }

        // Store the raw index instead of the floating point frequency.
        // It is much easier and safer to find the "mode" of integers!
        recorded_peaks[fft_run_count] = highest_freq_index;
        float f_found = (float)highest_freq_index * (current_fft_rate / (float)SAMPLES);
        fft_run_count++;

        Serial.print("Calibration Run ");
        Serial.print(fft_run_count);
        Serial.print("/");
        Serial.print(MAX_FFT_RUNS);
        Serial.print("\tPeak: ");
        Serial.print(f_found);
        Serial.println(" Hz");


        // --- CALIBRATION COMPLETE LOGIC ---
        if (fft_run_count >= MAX_FFT_RUNS) {
          
          // 1. Find the Mode (The most frequent highest index)
          int mode_index = 0;
          int max_occurrences = 0;

          for (int i = 0; i < MAX_FFT_RUNS; i++) {
            int count = 0;
            for (int j = 0; j < MAX_FFT_RUNS; j++) {
              if (recorded_peaks[j] == recorded_peaks[i]) count++;
            }
            if (count > max_occurrences) {
              max_occurrences = count;
              mode_index = recorded_peaks[i];
            }
          }

          // 2. Convert the winning index to the actual frequency
          float f_max = (float)mode_index * (current_fft_rate / (float)SAMPLES);
          
          Serial.println("\n====================================");
          Serial.print("CALIBRATION FINISHED. F_MAX: ");
          Serial.print(f_max);
          Serial.println(" Hz");

          // 3. Set the sample rate to 2.2 * f_max
          float target_fft_rate = f_max * 2.25f; 
          
          // Apply safety bounds
          if (target_fft_rate < 4000.0f) target_fft_rate = 4000.0f; 
          if (target_fft_rate > 100000.0f) target_fft_rate = 100000.0f;
          
          uint32_t target_i2s_rate = (uint32_t)(target_fft_rate);
          
          i2s_set_sample_rates(I2S_NUM_0, target_i2s_rate);
          
          Serial.print("NEW HARDWARE SAMPLE RATE APPLIED: ");
          Serial.print(target_i2s_rate);
          Serial.println(" Hz");
          Serial.println("====================================\n");

          // 4. Trigger the State Change
          calibration_done = true;

          // 5. ERASE MEMORY AND COMMIT SUICIDE (Delete the Task)
          free(fft_data);
          free(window_coefficients);
          vQueueDelete(fftsampleQueue); // Erase the FreeRTOS queue
          
          Serial.println("Memory Freed. FFT Task Terminated. Moving to Sampler Run Phase.");
          vTaskDelete(NULL); // Deletes THIS task safely
        }

        sample_index = 0; 
      }
    }
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);

  // ALLOCATE MEMORY DYNAMICALLY ON THE HEAP
  // This takes up ~49 KB of RAM, but we get it all back later!
  fft_data = (float*) malloc(SAMPLES * 2 * sizeof(float));
  window_coefficients = (float*) malloc(SAMPLES * sizeof(float));

  dsps_fft2r_init_fc32(NULL, 4096);
  dsps_wind_hann_f32(window_coefficients, SAMPLES);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = current_i2s_rate, 
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

  fftsampleQueue = xQueueCreate(1, sizeof(uint16_t) * 64);

  xTaskCreatePinnedToCore(i2sReadTask, "I2S_Read", 4096, NULL, 2, &Task1Handle, 0);
  xTaskCreatePinnedToCore(fftTask, "FFT_Process", 8192, NULL, 3, &FFTTaskHandle, 1);
}

void loop() {
  vTaskDelete(NULL); 
}