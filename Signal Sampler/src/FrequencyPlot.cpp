#include <Arduino.h>
#include "driver/i2s.h"

#define I2S_NUM         I2S_NUM_0
#define SAMPLE_RATE     44100

// Heltec V3 Pins
#define I2S_BCLK_PIN    4
#define I2S_LRCK_PIN    5
#define I2S_DIN_PIN     6

void setup() {
  // Lowered to standard 115200 baud
  Serial.begin(115200); 
  while (!Serial) { delay(10); }

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256, 
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_BCLK_PIN,
      .ws_io_num = I2S_LRCK_PIN,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_DIN_PIN
  };

  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
  i2s_start(I2S_NUM);
}

void loop() {
  // Read a block of 500 samples. 
  // At 44,100 Hz, it takes exactly 11.3 milliseconds for the hardware to fill this.
  const int CHUNK_SIZE = 500;
  uint16_t buffer[CHUNK_SIZE];
  size_t bytes_read;

  // 1. Read continuously. NO MORE i2s_stop() or i2s_start()!
  // This keeps the DMA perfectly aligned.
  i2s_read(I2S_NUM, &buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);

  // 2. DECIMATION: We only plot the FIRST sample out of the 500.
  // This reduces our Serial output from 44,100 lines/sec down to a safe ~88 lines/sec.
  // A 2 Hz wave plotted at 88 Hz will look incredibly smooth, and it easily fits within 115200 baud!
//   Serial.print("SineWave:");
  Serial.println(buffer[0]); 

  // 3. NO DELAY. We loop immediately to catch the next chunk.
}