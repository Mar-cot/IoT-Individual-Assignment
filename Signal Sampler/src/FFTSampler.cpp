#include "driver/i2s.h"
#include "driver/adc.h"       
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"    
#include "esp_dsp.h" 
#include "secrets.h"
#include <WiFi.h>             
#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>
#include "esp_timer.h" 
#include "esp_sleep.h"
#include "soc/rtc_wdt.h"
#include "esp_wifi.h"

#define DECIMATION_FACTOR 2
#define SAMPLES 4096
#define BUFFER_LEN 64
#define MAX_FFT_RUNS 15 
#define AGGREGATE_WINDOW_SECONDS 5 

uint32_t current_sample_rate = 100000; 
float current_fft_rate = (float)current_sample_rate / DECIMATION_FACTOR;
uint32_t aggregate_samples_needed;

QueueHandle_t fftsampleQueue;
QueueHandle_t processingQueue; 
QueueHandle_t aggregateQueue;  

TaskHandle_t Task1Handle;
TaskHandle_t FFTTaskHandle;
TaskHandle_t ProcessingTaskHandle; 
TaskHandle_t ConsumerTaskHandle; 
TaskHandle_t AnalogReadTaskHandle; 

float *fft_data; 
float *window_coefficients;

volatile bool calibration_done = false;
volatile bool hardware_rebuilding = false;
int calibration_stage = 0;
uint32_t sleep_count = 0;

// THE INTERLOCK: Prevents hardware sleep while Wi-Fi is booting/transmitting
volatile bool critical_task_running = false; 

// --- THINGSBOARD GLOBALS ---
WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient); 
ThingsBoard tb(mqttClient);

// ---------------------------------------------------------
// DYNAMIC NETWORK CONTROLLERS
// ---------------------------------------------------------
void connectNetwork() {
  critical_task_running = true; // Lock out hardware sleep

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  Serial.print("\nRadio ON. Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(200));
    Serial.print(".");
  }

  Serial.print(" Connecting to ThingsBoard...");
  while (!tb.connected()) {
    if (tb.connect(THINGSBOARD_SERVER, DEVICE_TOKEN)) {
      Serial.println(" Connected!");
    } else {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void disconnectNetwork() {
  // Give MQTT a fraction of a second to ensure the packet leaves the antenna
  vTaskDelay(pdMS_TO_TICKS(100)); 
  
  tb.disconnect();
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF); // Physically cuts power to the RF synthesizer
  
  Serial.println("Radio OFF. Resuming Light Sleep Architecture.");
  
  critical_task_running = false; // Allow hardware sleep again
}

// ---------------------------------------------------------
// HIGH-SPEED PATH: I2S DMA
// ---------------------------------------------------------
void i2sReadTask(void *pvParameters) {
  uint16_t buffer[BUFFER_LEN];
  size_t bytes_read;

  for (;;) {
    if (hardware_rebuilding) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    i2s_read(I2S_NUM_0, buffer, sizeof(buffer), &bytes_read, 500);
    if(bytes_read == 0) continue;
    
    if (!calibration_done) {
      // Reverted to Overwrite
      xQueueOverwrite(fftsampleQueue, buffer);
    } else {
      xQueueSend(processingQueue, buffer, portMAX_DELAY);
    }
  }
}

// ---------------------------------------------------------
// LOW-SPEED PATH: NATIVE ANALOG READ (Microsecond Precision)
// ---------------------------------------------------------
void analogReadTask(void *pvParameters) {
  uint16_t buffer[BUFFER_LEN];
  int buffer_idx = 0;

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);

  int64_t next_sample_time = esp_timer_get_time();

  for (;;) {
    int64_t sample_interval_us = 1000000 / current_sample_rate;
    int64_t now = esp_timer_get_time();
    int64_t time_to_wait_us = next_sample_time - now;

    // --- HARDWARE SLEEP INTERLOCK LOGIC ---
    if (calibration_done && time_to_wait_us > 1000) {
      if (!critical_task_running) {
        esp_sleep_enable_timer_wakeup(time_to_wait_us - 400);
        sleep_count++;
        esp_light_sleep_start();
      } 
      else if (time_to_wait_us > 2000) {
        vTaskDelay(pdMS_TO_TICKS(time_to_wait_us / 1000) - 1);
      }
    } else if (time_to_wait_us > 2000) {
      vTaskDelay(pdMS_TO_TICKS(time_to_wait_us / 1000) - 1);
    }

    while (esp_timer_get_time() < next_sample_time) {
      taskYIELD(); 
    }

    next_sample_time += sample_interval_us;

    buffer[buffer_idx++] = adc1_get_raw(ADC1_CHANNEL_6);

    if (buffer_idx >= BUFFER_LEN) {
      buffer_idx = 0;
      
      if (!calibration_done) {
        // Reverted to Overwrite
        xQueueOverwrite(fftsampleQueue, buffer);
      } else {
        xQueueSend(processingQueue, buffer, portMAX_DELAY);
      }
    }
  }
}

// ---------------------------------------------------------
// HYBRID DSP ENGINE
// ---------------------------------------------------------
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

        Serial.print(calibration_stage == 0 ? "I2S Sweep " : "Analog Sweep ");
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
          Serial.print("STAGE "); Serial.print(calibration_stage);
          Serial.print(" FINISHED. F_MAX: ");
          Serial.print(f_max); Serial.println(" Hz");

          if (calibration_stage == 0) {
            float target_fft_rate = f_max * 2.1f;
            
            if (target_fft_rate >= 4000.0f) {
              if (target_fft_rate > 62500.0f) target_fft_rate = 62500.0f;
              
              current_sample_rate = (uint32_t)(target_fft_rate * DECIMATION_FACTOR); 

              hardware_rebuilding = true;
              vTaskDelay(pdMS_TO_TICKS(600)); 

              i2s_stop(I2S_NUM_0);
              i2s_driver_uninstall(I2S_NUM_0);

              i2s_config_t i2s_config = {
                .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
                .sample_rate = current_sample_rate, 
                .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
                .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,       
                .communication_format = I2S_COMM_FORMAT_STAND_I2S, 
                .intr_alloc_flags = 0,
                .dma_buf_count = 8,
                .dma_buf_len = BUFFER_LEN,
                .use_apll = true 
              };

              i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
              i2s_set_pin(I2S_NUM_0, NULL);
              i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_6);
              hardware_rebuilding = false;
              
              Serial.print("NEW HARDWARE SAMPLE RATE APPLIED: ");
              Serial.print(current_sample_rate); Serial.println(" Hz");
              
              aggregate_samples_needed = current_sample_rate * AGGREGATE_WINDOW_SECONDS;
              calibration_done = true;

              free(fft_data);
              free(window_coefficients);
              vQueueDelete(fftsampleQueue); 
              vTaskDelete(NULL); 
              
            } else {
              Serial.println(">>> Target < 4000Hz! Tearing down I2S and switching to precision AnalogRead...");
              calibration_stage = 1;
              
              hardware_rebuilding = true;
              vTaskDelay(pdMS_TO_TICKS(100)); 
              vTaskDelete(Task1Handle); 
              
              i2s_stop(I2S_NUM_0);
              i2s_driver_uninstall(I2S_NUM_0);
              
              current_sample_rate = 5000;
              current_fft_rate = (float)current_sample_rate / DECIMATION_FACTOR;
              
              fft_run_count = 0;
              sample_index = 0;
              xQueueReset(fftsampleQueue);
              
              xTaskCreatePinnedToCore(analogReadTask, "Analog_Read", 4096, NULL, 2, &AnalogReadTaskHandle, 0);
            }
          } 
          else if (calibration_stage == 1) {
            float target_fft_rate = f_max * 2.1f;
            
            if (target_fft_rate < 20.0f) target_fft_rate = 20.0f; 
            if (target_fft_rate > 5000.0f) target_fft_rate = 5000.0f; 
            
            current_sample_rate = (uint32_t)(target_fft_rate); 
            
            Serial.print("NEW ANALOG SOFTWARE RATE APPLIED: ");
            Serial.print(current_sample_rate); Serial.println(" Hz");
            Serial.println("====================================\n");

            aggregate_samples_needed = current_sample_rate * AGGREGATE_WINDOW_SECONDS;
            
            calibration_done = true;
            
            free(fft_data);
            free(window_coefficients);
            vQueueDelete(fftsampleQueue); 
            vTaskDelete(NULL); 
          }
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
  uint64_t _delta_time = 0;
  uint64_t _start_time = esp_timer_get_time();
  uint64_t _end_time = 0;

  for (;;) {
    if (!calibration_done) {
      vTaskDelay(pdMS_TO_TICKS(100)); 
      continue;
    }

    if (xQueueReceive(processingQueue, rx_buffer, portMAX_DELAY) == pdPASS) {
      for (int i = 0; i < BUFFER_LEN; i++) {
        
        total_sum += (rx_buffer[i] & 0xFFF); 
        samples_passed++;
        
        if (samples_passed >= aggregate_samples_needed) {
          float average = (float)total_sum / (float)samples_passed;
          
          _end_time = esp_timer_get_time();
          
          xQueueSend(aggregateQueue, &average, portMAX_DELAY);
          
          _delta_time += _end_time - _start_time;
          Serial.print("Window execution time: "); Serial.print(_delta_time); Serial.println("us");
          _delta_time = 0;
          total_sum = 0;
          samples_passed = 0;
          _start_time = esp_timer_get_time();
        }
      }
    }
  }
}

void aggregateConsumerTask(void *pvParameters) {
  float final_average;
  uint64_t _start_time = 0, _end_time = 0, _delta_time = 0;

  // Make sure radio is completely off to start
  WiFi.mode(WIFI_OFF); 

  for (;;) {
    if (xQueueReceive(aggregateQueue, &final_average, portMAX_DELAY) == pdPASS) {
      _start_time = esp_timer_get_time();

      connectNetwork(); 

      tb.sendTelemetryData("average_value", final_average);
      tb.loop();
      disconnectNetwork(); 
      _end_time = esp_timer_get_time();
      _delta_time += _end_time - _start_time;

      Serial.print(">>> [THINGSBOARD PUBLISHED] average_value: ");
      Serial.print(final_average);
      Serial.print(" \t delta_time: ");
      Serial.print(_delta_time);
      Serial.print("\t sleep_count: ");
      Serial.println(sleep_count);
      

      sleep_count = 0;
      _delta_time = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  fft_data = (float*) malloc(SAMPLES * 2 * sizeof(float));
  window_coefficients = (float*) malloc(SAMPLES * sizeof(float));

  disableCore0WDT(); 
  dsps_fft2r_init_fc32(NULL, 4096);
  dsps_wind_hann_f32(window_coefficients, SAMPLES);

  // wifi_config_t conf;
  // esp_wifi_get_config(WIFI_IF_STA, &conf);
  // conf.sta.listen_interval = 3; 
  // esp_wifi_set_config(WIFI_IF_STA, &conf);

  // esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  // esp_sleep_enable_wifi_wakeup();

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = current_sample_rate, 
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,       
    .communication_format = I2S_COMM_FORMAT_STAND_I2S, 
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_LEN,
    .use_apll = true 
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_6);

  // REVERTED to size 1 
  fftsampleQueue = xQueueCreate(1, sizeof(uint16_t) * BUFFER_LEN);
  processingQueue = xQueueCreate(10, sizeof(uint16_t) * BUFFER_LEN);
  aggregateQueue = xQueueCreate(10, sizeof(float)); 

  xTaskCreatePinnedToCore(i2sReadTask, "I2S_Read", 4096, NULL, 2, &Task1Handle, 0);
  xTaskCreatePinnedToCore(fftTask, "FFT_Process", 8192, NULL, 3, &FFTTaskHandle, 1);
  xTaskCreatePinnedToCore(processingTask, "Data_Processing", 4096, NULL, 3, &ProcessingTaskHandle, 1);
  xTaskCreatePinnedToCore(aggregateConsumerTask, "TB_Consumer", 4096, NULL, 2, &ConsumerTaskHandle, 1);
}

void loop() {
  vTaskDelete(NULL); 
}