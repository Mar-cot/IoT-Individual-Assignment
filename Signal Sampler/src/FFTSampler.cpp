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

uint32_t aggregate_samples_needed;


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

volatile bool hardware_rebuilding = false;


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

#include "soc/i2s_reg.h"
//ONLY WORKS IF .use_apll = false;
/* float get_actual_hardware_sample_rate() {
  // 1. Read the Clock Master (CLKM) configuration register
  uint32_t clkm_conf = REG_READ(I2S_CLKM_CONF_REG(0));
  
  // Extract the integer and fractional dividers
  uint32_t clkm_div_num = (clkm_conf >> I2S_CLKM_DIV_NUM_S) & I2S_CLKM_DIV_NUM_V;
  uint32_t clkm_div_b   = (clkm_conf >> I2S_CLKM_DIV_B_S) & I2S_CLKM_DIV_B_V;
  uint32_t clkm_div_a   = (clkm_conf >> I2S_CLKM_DIV_A_S) & I2S_CLKM_DIV_A_V;

  // 2. Read the Bit Clock (BCLK) configuration register
  uint32_t sample_conf = REG_READ(I2S_SAMPLE_RATE_CONF_REG(0));
  uint32_t rx_bck_div_num = (sample_conf >> I2S_RX_BCK_DIV_NUM_S) & I2S_RX_BCK_DIV_NUM_V;

  // Prevent divide by zero errors if the hardware isn't fully initialized yet
  if (clkm_div_a == 0) clkm_div_a = 1; 
  if (rx_bck_div_num == 0) rx_bck_div_num = 1; 

  // 3. Reconstruct the exact fractional multiplier the silicon is using
  float clkm_div = (float)clkm_div_num + ((float)clkm_div_b / (float)clkm_div_a);

  // 4. Calculate the final sample rate
  // Base clock for I2S (when use_apll = false) is the 160 MHz system PLL
  // 32 is the I2S frame size (16 bits per channel * 2 channels)
  float actual_rate = 160000000.0f / clkm_div / (float)rx_bck_div_num / 32.0f;

  return actual_rate;
}
 */

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
          if (target_fft_rate < 725.0f) target_fft_rate = 725.0f; // Empirically proven that below 725Hz the clock bleeds, hard mathematical limit is ~653.59Hz 
          if (target_fft_rate > 62500.0f) target_fft_rate = 62500.0f; // Due to how FFT is implemented, f_max <= 25kHz 
          
          current_i2s_rate = (uint32_t)(target_fft_rate); 

          // --- 1. SIGNAL THE READER TASK TO PAUSE ---
          hardware_rebuilding = true;
          // Give Core 0 enough time to finish its current read cycle and pause
          vTaskDelay(pdMS_TO_TICKS(600)); 

          // --- 2. SAFELY REBUILD HARDWARE ---
          i2s_stop(I2S_NUM_0);
          i2s_driver_uninstall(I2S_NUM_0);

          i2s_config_t i2s_config = {
            .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
            .sample_rate = current_i2s_rate, 
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
          // i2s_adc_enable(I2S_NUM_0);
          
          // --- 3. RESUME READING ---
          hardware_rebuilding = false;
          
          Serial.print("NEW HARDWARE SAMPLE RATE APPLIED: ");
          Serial.print(current_i2s_rate); Serial.println(" Hz");
          Serial.println("====================================\n");

          aggregate_samples_needed = current_i2s_rate*AGGREGATE_WINDOW_SECONDS;
          Serial.print("Number of samples to aggregate: ");
          Serial.println(aggregate_samples_needed);
          Serial.println("====================================\n");

          vTaskDelay(pdMS_TO_TICKS(1000));

          // ONLY WORKS IF .use_apll = false;
          /* float real_hardware_rate = get_actual_hardware_sample_rate();
          
          Serial.println("\n--- CLOCK VERIFICATION ---");
          Serial.print("Software Requested: ");
          Serial.print(current_i2s_rate);
          Serial.println(" Hz");
          
          Serial.print("Silicon Actual:     ");
          Serial.print(real_hardware_rate*16);
          Serial.println(" Hz");
          Serial.println("--------------------------\n");
 */
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
      

      for (int i = 0; i < BUFFER_LEN; i++) {
        
        total_sum += (rx_buffer[i] & 0xFFF); 
        samples_passed++;

        if (samples_passed >= aggregate_samples_needed) {
          
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

/* void LightSleepMeasuringTask(void *pvParameters) {
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
} */

void aggregateConsumerTask(void *pvParameters) {
  float final_average;
  
  // Record the baseline time before entering the infinite loop
  unsigned long last_publish_time = millis(); 

  for (;;) {
    if (!tb.connected()) {
      connectToNetwork();
    }
    
    tb.loop();

    // 100ms timeout allows the task to breathe and hit tb.loop() to maintain connection
    if (xQueueReceive(aggregateQueue, &final_average, pdMS_TO_TICKS(100)) == pdPASS) {
      
      // Calculate how much time has passed since the last successful receive/publish
      unsigned long current_time = millis();
      unsigned long delta_time = current_time - last_publish_time;
      last_publish_time = current_time; // Update the baseline for the next run

      // The ThingsBoard library handles JSON serialization automatically
      tb.sendTelemetryData("average_value", final_average);

      // Print the new formatted output with the delta time
      Serial.print(">>> [THINGSBOARD PUBLISHED] average_value: ");
      Serial.print(final_average);
      Serial.print(" \t delta_time: ");
      Serial.print(delta_time);
      Serial.println(" ms");
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
    .dma_buf_len = BUFFER_LEN,
    .use_apll = true 
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_6);
  // i2s_adc_enable(I2S_NUM_0);

  fftsampleQueue = xQueueCreate(1, sizeof(uint16_t) * 64);
  processingQueue = xQueueCreate(1, sizeof(uint16_t) * 64);
  aggregateQueue = xQueueCreate(10, sizeof(float)); 

  xTaskCreatePinnedToCore(i2sReadTask, "I2S_Read", 4096, NULL, 2, &Task1Handle, 0);
  xTaskCreatePinnedToCore(fftTask, "FFT_Process", 8192, NULL, 3, &FFTTaskHandle, 1);
  xTaskCreatePinnedToCore(processingTask, "Data_Processing", 4096, NULL, 3, &ProcessingTaskHandle, 1);
  xTaskCreatePinnedToCore(aggregateConsumerTask, "TB_Consumer", 4096, NULL, 2, &ConsumerTaskHandle, 1);
  // xTaskCreatePinnedToCore(LightSleepMeasuringTask, "Sleep_Test", 2048, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelete(NULL); 
}