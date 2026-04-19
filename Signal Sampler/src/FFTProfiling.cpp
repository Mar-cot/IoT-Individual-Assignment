#include <Arduino.h>
#include "driver/i2s.h"
#include <arduinoFFT.h>

// --- Configuration ---
#define I2S_NUM         I2S_NUM_0
#define I2S_BCLK_PIN    4
#define I2S_LRCK_PIN    5
#define I2S_DIN_PIN     6

#define PROFILING_RATE  200000 // 200 kHz Max Oversampling
#define FFT_SAMPLES     2048*4   // Must be power of 2. Uses ~32KB RAM.

// FFT Arrays
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];

// Create FFT object
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, PROFILING_RATE);

// State Machine
enum SamplerState {
    STATE_PROFILING,
    STATE_RECONFIGURING,
    STATE_PLOTTING
};
SamplerState currentState = STATE_PROFILING;
uint32_t active_sample_rate = PROFILING_RATE;

// ---------------------------------------------------------
// Reusable I2S Installer (Heltec is MASTER)
// ---------------------------------------------------------
void install_i2s(uint32_t sample_rate) {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = sample_rate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // Adjust to LEFT if using standard I2S mic
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

// ---------------------------------------------------------
// MAIN SETUP
// ---------------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println("--- Booting Smart Sampler ---");
    Serial.printf("Phase 1: Oversampling at %lu Hz...\n", PROFILING_RATE);
    
    install_i2s(PROFILING_RATE);
}

// ---------------------------------------------------------
// MAIN LOOP (State Machine)
// ---------------------------------------------------------
void loop() {
    
    // =======================================================
    // STEP 1 & 2: OVERSAMPLE & FFT PROFILING
    // =======================================================
    if (currentState == STATE_PROFILING) {
        uint16_t raw_sample = 0;
        size_t bytes_read = 0;

        // Fill the FFT arrays with exactly 2048 samples
        for (int i = 0; i < FFT_SAMPLES; i++) {
            i2s_read(I2S_NUM, &raw_sample, sizeof(raw_sample), &bytes_read, portMAX_DELAY);
            vReal[i] = (double)raw_sample; 
            vImag[i] = 0.0; 
        }

        Serial.println("Data collected. Computing FFT...");

        // Perform FFT
        FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(FFT_FORWARD);
        FFT.complexToMagnitude();

        // Find highest significant frequency (Start at bin 5 to ignore DC offset/0 Hz)
        double peak_freq = 0;
        double peak_magnitude = 0;
        
        for (int i = 5; i < (FFT_SAMPLES / 2); i++) {
            if (vReal[i] > 0) Serial.println(String(i) + ": " + String(vReal[i]));
            if (vReal[i] > peak_magnitude) {
                peak_magnitude = vReal[i];
                peak_freq = ((double)i * PROFILING_RATE) / FFT_SAMPLES; 
            }
        }

        Serial.printf("FFT Complete. Dominant Frequency: %.2f Hz\n", peak_freq);
        
        // Store the result and move to next state
        active_sample_rate = (uint32_t)peak_freq;
        currentState = STATE_RECONFIGURING;
    }

    // =======================================================
    // STEP 3: ADJUST SAMPLING (NYQUIST + MARGIN)
    // =======================================================
    else if (currentState == STATE_RECONFIGURING) {
        
        // Apply Nyquist theorem (f_sample > 2 * f_signal). 
        // We multiply by 2.2 for a safe 10% anti-aliasing margin.
        uint32_t new_rate = (uint32_t)(active_sample_rate * 2.2);

        // Hardware safety checks
        if (new_rate < 8000) { new_rate = 8000; } // Minimum stable I2S speed
        if (new_rate > PROFILING_RATE) { new_rate = PROFILING_RATE; } 

        Serial.printf("Phase 3: Nyquist Adjustment. Reconfiguring I2S to %lu Hz\n", new_rate);

        // Tear down the old driver, install the new one
        i2s_stop(I2S_NUM);
        i2s_driver_uninstall(I2S_NUM);
        install_i2s(new_rate);

        // Update tracking variable and move to plotting
        active_sample_rate = new_rate;
        Serial.println("Phase 4: Entering Live Plotting Mode...");
        vTaskDelay(1000); // Brief pause so you can read the console before plotting starts
        
        currentState = STATE_PLOTTING;
    }

    // =======================================================
    // STEP 4: PLOT THE SIGNAL (OSCILLOSCOPE MODE)
    // =======================================================
    else if (currentState == STATE_PLOTTING) {
        
        // We read small bursts to keep BetterSerialPlotter happy at 115200 baud
        const int BURST_SIZE = 100;
        uint16_t buffer[BURST_SIZE];
        size_t bytes_read;

        // "Flash Photography" method: Stop and start to guarantee flawless DMA alignment
        i2s_stop(I2S_NUM);
        i2s_start(I2S_NUM);

        // Read burst
        i2s_read(I2S_NUM, &buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);

        // Print pure numbers for BetterSerialPlotter
        for (int i = 0; i < BURST_SIZE; i++) {
            // Serial.println(buffer[i]);
        }

        // Delay to allow UART buffer to clear (~100ms is safe for 100 samples at 115200)
        vTaskDelay(100); 
    }
}