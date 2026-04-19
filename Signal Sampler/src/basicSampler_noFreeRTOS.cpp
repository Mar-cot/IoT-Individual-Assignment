#include "driver/i2s.h"
#include <Arduino.h>

#define SAMPLE_RATE 3000
#define I2S_NUM     I2S_NUM_0

void setup() {
  Serial.begin(115200);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,       // ADC natively uses Left
    .communication_format = I2S_COMM_FORMAT_STAND_I2S, // Revert back to standard
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = true // Setting to true uses the much cleaner Audio PLL clock
    };

  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);

  i2s_set_pin(I2S_NUM, NULL);

  i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_6); // GPIO34
  i2s_adc_enable(I2S_NUM);
}

void loop() {
  uint16_t buffer[64];
  size_t bytes_read;

  // Read a block of samples from the DMA buffer
  i2s_read(I2S_NUM, &buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);

  // Print just the first sample of each block to Serial  (to avoid flooding)
  for (size_t i = 0; i < 64; i++)
  {
    /* code */
    Serial.println(buffer[i]&0xFFF); // Mask 12 bits for ADC
  }
  
}