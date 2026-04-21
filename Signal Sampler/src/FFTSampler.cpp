#include "driver/i2s.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_dsp.h" 
#include "secrets.h"
#include <WiFi.h>             
#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>
#include "esp32/clk.h"

#define DECIMATION_FACTOR 2
#define SAMPLES 4096
#define BUFFER_LEN 64
#define MAX_FFT_RUNS 50 
#define AGGREGATE_WINDOW_SECONDS 5 


uint32_t current_i2s_rate = 100000; 
float current_fft_rate = (float)current_i2s_rate / DECIMATION_FACTOR; 

QueueHandle_t fftsampleQueue;
QueueHandle_t processingQueue; 
QueueHandle_t aggregateQueue;  

TaskHandle_t Task1Handle;
TaskHandle_t FFTTaskHandle;
TaskHandle_t ProcessingTaskHandle; 
TaskHandle_t ConsumerTaskHandle; 

float *fft_data; 
float *window_coefficients;
volatile bool calibration_done = false;

// --- THINGSBOARD GLOBALS ---
WiFiClient espClient;
// Wrap the raw WiFi client in the ThingsBoard MQTT interface
Arduino_MQTT_Client mqttClient(espClient); 
// Pass the MQTT client wrapper to ThingsBoard
ThingsBoard tb(mqttClient);
void connectToNetwork() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(500));
      Serial.print(".");
    }
    Serial.println("\nWiFi Connected.");
  }

  while (!tb.connected()) {
    Serial.print("Connecting to ThingsBoard node...");
    if (tb.connect(THINGSBOARD_SERVER, DEVICE_TOKEN)) {
      Serial.println("\nThingsBoard Connected.");
    } else {
      Serial.print("\nFailed to connect. Retrying in 5 seconds...");
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}

void i2sReadTask(void *pvParameters) {
  uint16_t buffer[BUFFER_LEN];
  size_t bytes_read;

  for (;;) {
    i2s_read(I2S_NUM_0, buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
    
    if (!calibration_done) {
      xQueueOverwrite(fftsampleQueue, buffer);
    } else {
      xQueueOverwrite(processingQueue, buffer);
    }
  }
}

void fftTask(void *pvParameters) {
  uint16_t rx_buffer[BUFFER_LEN];
  int sample_index = 0;
  
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

        float max_magnitude = 0;
        
        for (int i = 1; i < SAMPLES / 2; i++) {
          float real = fft_data[i * 2];
          float imag = fft_data[i * 2 + 1];
          float magnitude = sqrt((real * real) + (imag * imag));
          fft_data[i] = magnitude; 
          if (magnitude > max_magnitude) max_magnitude = magnitude;
        }

        float noise_threshold = max_magnitude * 0.05f; 
        int highest_freq_index = 0;

        for (int i = (SAMPLES / 2) - 1; i >= 1; i--) {
          if (fft_data[i] >= noise_threshold) {
            highest_freq_index = i;
            break; 
          }
        }

        recorded_peaks[fft_run_count] = highest_freq_index;
        float f_found = (float)highest_freq_index * (current_fft_rate / (float)SAMPLES);
        fft_run_count++;

        Serial.print("Calibration Run ");
        Serial.print(fft_run_count); Serial.print("/"); Serial.print(MAX_FFT_RUNS);
        Serial.print("\tPeak: "); Serial.print(f_found); Serial.println(" Hz");

        if (fft_run_count >= MAX_FFT_RUNS) {
          
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

          float f_max = (float)mode_index * (current_fft_rate / (float)SAMPLES);
          
          Serial.println("\n====================================");
          Serial.print("CALIBRATION FINISHED. F_MAX: ");
          Serial.print(f_max); Serial.println(" Hz");

          float target_fft_rate = f_max * 2.25f; 
          if (target_fft_rate < 4000.0f) target_fft_rate = 4000.0f; 
          if (target_fft_rate > 100000.0f) target_fft_rate = 100000.0f;
          
          current_i2s_rate = (uint32_t)(target_fft_rate); 
          i2s_set_sample_rates(I2S_NUM_0, current_i2s_rate);
          
          Serial.print("NEW HARDWARE SAMPLE RATE APPLIED: ");
          Serial.print(current_i2s_rate); Serial.println(" Hz");
          Serial.println("====================================\n");

          calibration_done = true;

          free(fft_data);
          free(window_coefficients);
          vQueueDelete(fftsampleQueue); 
          
          Serial.println("Memory Freed. FFT Task Terminated. Moving to Sampler Run Phase.");
          vTaskDelete(NULL); 
        }
        sample_index = 0; 
      }
    }
  }
}

void processingTask(void *pvParameters) {
  uint16_t rx_buffer[BUFFER_LEN];
  uint64_t total_sum = 0; 
  uint32_t samples_passed = 0;

  for (;;) {
    if (!calibration_done) {
      vTaskDelay(pdMS_TO_TICKS(100)); 
      continue;
    }

    if (xQueueReceive(processingQueue, rx_buffer, portMAX_DELAY) == pdPASS) {
      
      uint32_t samples_needed = current_i2s_rate * AGGREGATE_WINDOW_SECONDS;

      for (int i = 0; i < BUFFER_LEN; i++) {
        
        total_sum += (rx_buffer[i] & 0xFFF); 
        samples_passed++;

        if (samples_passed >= samples_needed) {
          
          float average = (float)total_sum / (float)samples_passed;
          xQueueSend(aggregateQueue, &average, portMAX_DELAY);

          total_sum = 0;
          samples_passed = 0;
        }
      }
    }
  }
}

#include "esp_sleep.h"
#include "esp_timer.h" 

void LightSleepMeasuringTask(void *pvParameters) {
  // First wait: 20 seconds to let Wi-Fi, MQTT, and the first FFT calibrations settle
  vTaskDelay(pdMS_TO_TICKS(20000)); 

  for(;;) {
    // Wait 10 seconds between sleep tests
    vTaskDelay(pdMS_TO_TICKS(10000)); 

    Serial.println("\n>>> Entering Minimum Light Sleep...");

    // 1. Set the wakeup timer to the absolute minimum: 1 microsecond
    esp_sleep_enable_timer_wakeup(1);

    // 2. Grab the high-resolution hardware timestamp right before sleep
    int64_t time_before = esp_timer_get_time();

    // 3. Trigger Light Sleep
    // The CPU halts execution exactly on this line. RAM is preserved.
    esp_light_sleep_start();

    // 4. The CPU wakes up and resumes execution exactly on the next line!
    int64_t time_after = esp_timer_get_time();

    // 5. Calculate the true hardware overhead
    int64_t delta_us = time_after - time_before;
    float delta_ms = (float)delta_us / 1000.0f;

    Serial.print(">>> Woke up! Real elapsed time: ");
    Serial.print(delta_us);
    Serial.print(" us (");
    Serial.print(delta_ms);
    Serial.println(" ms)");
  }
}

void aggregateConsumerTask(void *pvParameters) {
  float final_average;

  for (;;) {
    if (!tb.connected()) {
      connectToNetwork();
    }
    
    tb.loop();

    // 100ms timeout allows the task to breathe and hit tb.loop() to maintain connection
    if (xQueueReceive(aggregateQueue, &final_average, pdMS_TO_TICKS(100)) == pdPASS) {
      
      // The ThingsBoard library handles JSON serialization automatically
      tb.sendTelemetryData("average_value", final_average);

      Serial.print(">>> [THINGSBOARD PUBLISHED] average_value: ");
      Serial.println(final_average);
    }
  }
}

void setup() {
  Serial.begin(115200);

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
  processingQueue = xQueueCreate(1, sizeof(uint16_t) * 64);
  aggregateQueue = xQueueCreate(10, sizeof(float)); 

  xTaskCreatePinnedToCore(i2sReadTask, "I2S_Read", 4096, NULL, 2, &Task1Handle, 0);
  xTaskCreatePinnedToCore(fftTask, "FFT_Process", 8192, NULL, 3, &FFTTaskHandle, 1);
  xTaskCreatePinnedToCore(processingTask, "Data_Processing", 4096, NULL, 3, &ProcessingTaskHandle, 1);
  xTaskCreatePinnedToCore(aggregateConsumerTask, "TB_Consumer", 4096, NULL, 2, &ConsumerTaskHandle, 1);
  xTaskCreatePinnedToCore(LightSleepMeasuringTask, "Sleep_Test", 2048, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelete(NULL); 
}