#include "driver/i2s.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define SAMPLE_RATE 44100
#define I2S_NUM     I2S_NUM_0

// FreeRTOS Handles
QueueHandle_t sampleQueue;
TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;

// ---------------------------------------------------------
// TASK 1: Read I2S (Pinned to Core 0)
// ---------------------------------------------------------
void i2sReadTask(void *pvParameters) {
  uint16_t buffer[64];
  size_t bytes_read;

  for (;;) {
    // Read a block of samples from the DMA buffer
    i2s_read(I2S_NUM, buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);

    // Overwrite the queue with the newest buffer. 
    // xQueueOverwrite does not block and requires a queue length of exactly 1.
    xQueueOverwrite(sampleQueue, buffer);
  }
}

// ---------------------------------------------------------
// TASK 2: Print to Serial (Pinned to Core 1)
// ---------------------------------------------------------
void serialPrintTask(void *pvParameters) {
  uint16_t rx_buffer[64];
  for (;;) {
    // Wait for new data to arrive in the queue
    if (xQueueReceive(sampleQueue, rx_buffer, portMAX_DELAY) == pdPASS) {
      Serial.println(rx_buffer[0] & 0xFFF); // Mask 12 bits for ADC
      
    //   // Print the buffer
    //   for (size_t i = 0; i < 64; i++) {
    //     Serial.println(rx_buffer[i] & 0xFFF); // Mask 12 bits for ADC
    //   }
      
    }
  }
}

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {
  Serial.begin(256000);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,       
    .communication_format = I2S_COMM_FORMAT_STAND_I2S, 
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = true 
  };

  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_6); // GPIO34
  i2s_adc_enable(I2S_NUM);

  // 1. Create the Queue: Length of 1, Size of the entire 64-element array
  sampleQueue = xQueueCreate(1, sizeof(uint16_t) * 64);

  // 2. Create Task 1 (Reading) on Core 0
  xTaskCreatePinnedToCore(
    i2sReadTask,      // Task function
    "I2S_Read_Task",  // Task name (for debugging)
    4096*4,             // Stack size (bytes)
    NULL,             // Parameters passed to task
    2,                // Priority (higher number = higher priority)
    &Task1Handle,     // Task handle
    0                 // Core ID (Core 0)
  );

  // 3. Create Task 2 (Printing) on Core 1
  xTaskCreatePinnedToCore(
    serialPrintTask,    // Task function
    "Serial_Print_Task",// Task name 
    4096*4,               // Stack size
    NULL,               // Parameters
    1,                  // Priority (slightly lower than reading)
    &Task2Handle,       // Task handle
    1                   // Core ID (Core 1)
  );
}

// ---------------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------------
void loop() {
  // Since our FreeRTOS tasks run infinitely on their own, 
  // we can delete the default Arduino loop task to save resources.
  vTaskDelete(NULL); 
}