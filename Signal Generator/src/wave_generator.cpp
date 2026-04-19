#include "driver/i2s.h"
#include <math.h>
#include <Arduino.h>

#define I2S_NUM         I2S_NUM_0
// Generator Pins
#define I2S_BCLK_PIN    22
#define I2S_LRCK_PIN    23
#define I2S_DOUT_PIN    19
// We calculate the sine wave based on the Heltec's initial 200kHz speed
#define BASE_RATE       200000 
#define SINE_FREQ       2.0

void setup() {
  Serial.begin(115200);
  
  i2s_config_t i2s_config = {
    // CRITICAL: Switched back to SLAVE. It will listen to the Heltec's clocks.
    .mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_TX),
    .sample_rate = BASE_RATE, 
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_BCLK_PIN,
      .ws_io_num = I2S_LRCK_PIN,
      .data_out_num = I2S_DOUT_PIN,
      .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
  
  Serial.println("Generator Ready (SLAVE MODE). Waiting for Heltec Clocks...");
}

void loop() {
  static float phase = 0;
  uint16_t sample;
  size_t bytes_written;

  // Generate the sine wave value
  float val = sin(phase);
  sample = (uint16_t)((val + 1.0) * 32767.0); 

  // Push to DMA. This blocks infinitely until the Heltec asks for data!
  i2s_write(I2S_NUM, &sample, sizeof(sample), &bytes_written, portMAX_DELAY);

  // Advance the phase
  phase += 2 * PI * SINE_FREQ / BASE_RATE;
  if (phase >= 2 * PI) phase -= 2 * PI;
}