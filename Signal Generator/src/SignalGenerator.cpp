#include "driver/i2s.h"
#include <math.h>
#include <Arduino.h>

#define SAMPLE_RATE     100000 
#define I2S_NUM         I2S_NUM_0
#define WAVETABLE_SIZE  4096 
#define BUFFER_SIZE     64   

// 1. Define the Struct to hold dedicated arrays
struct Sinusoid {
  float amplitude;
  float target_frequency;
  float actual_frequency; // Adjusted so it loops perfectly in the buffer
  float precalculated_wave[WAVETABLE_SIZE]; // Dedicated array for this wave
};

// 2. Create your mix (Max 6 signals because of DRAM limitations)
Sinusoid active_signals[] = {
  // {1.0, 1000.0, 0.0, {}},
  // {1.0, 5000.0, 0.0, {}},
  // {0.3, 9700.0, 0.0, {}},  
  // {1.0, 200.0, 0.0, {}},
  {1.0, 50.0, 0.0, {}},
  // {1.0, 20.0, 0.0, {}},
  // {1.0, 5.0, 0.0, {}},
};

const int NUM_SIGNALS = sizeof(active_signals) / sizeof(active_signals[0]);
float total_amplitude = 0;

void setup() {
  Serial.begin(115200);

  // 1. Pre-calculate the exact time-domain points for each sinusoid
  for (int s = 0; s < NUM_SIGNALS; s++) {
    
    // Calculate how many EXACT cycles fit into our 4096 buffer
    float exact_cycles = (active_signals[s].target_frequency * WAVETABLE_SIZE) / SAMPLE_RATE;
    
    // Round to the nearest whole number of cycles to prevent popping/glitching when the array loops
    int whole_cycles = round(exact_cycles);
    
    // Calculate the actual frequency we must use to achieve perfect looping
    active_signals[s].actual_frequency = ((float)whole_cycles * SAMPLE_RATE) / WAVETABLE_SIZE;

    Serial.print("Target Hz: "); Serial.print(active_signals[s].target_frequency);
    Serial.print(" | Adjusted to: "); Serial.print(active_signals[s].actual_frequency);
    Serial.println(" Hz for seamless looping.");

    // Fill this specific sinusoid's array with its time-domain points
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
      float time_in_seconds = (float)i / SAMPLE_RATE;
      active_signals[s].precalculated_wave[i] = sin(2.0 * PI * active_signals[s].actual_frequency * time_in_seconds);
    }

    total_amplitude += active_signals[s].amplitude;
  }

  if (total_amplitude == 0) total_amplitude = 1.0;

  // --- I2S INITIALIZATION ---
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // FIX: Output to both channels
    .communication_format = I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false
  };

  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, NULL);
  i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN); 
}

void loop() {
  // FIX: Use 32-bit buffer to force I2S to consume 1 index = 1 true sample
  uint32_t out_buffer[BUFFER_SIZE];
  size_t bytes_written;
  
  // A global index to track where we are in our 4096-point arrays
  static int global_time_index = 0;

  for (int i = 0; i < BUFFER_SIZE; i++) {
    float sample_sum = 0;

    // Pick the point from each pre-calculated array and sum them
    for (int s = 0; s < NUM_SIGNALS; s++) {
      sample_sum += active_signals[s].precalculated_wave[global_time_index] * active_signals[s].amplitude;
    }

    // Normalize
    sample_sum /= total_amplitude;

    // Scale to DAC (0-254)
    uint16_t dac_val = (uint16_t)((sample_sum + 1.0) * 127.0);
    
    // Shift into MSB for DAC
    uint16_t dac_ready = dac_val << 8;

    // FIX: Pack the same 16-bit value into the Left and Right slots of the 32-bit frame
    out_buffer[i] = ((uint32_t)dac_ready << 16) | dac_ready;

    // Advance our global time tracker
    global_time_index++;
    if (global_time_index >= WAVETABLE_SIZE) {
      global_time_index = 0;
    }
  }

  i2s_write(I2S_NUM, out_buffer, sizeof(out_buffer), &bytes_written, portMAX_DELAY);
}