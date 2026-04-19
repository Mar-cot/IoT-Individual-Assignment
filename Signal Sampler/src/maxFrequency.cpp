#include <Arduino.h>
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ---------------------------------------------------------
// I2S Configuration & Pins (Adjust for your specific wiring)
// ---------------------------------------------------------
#define I2S_PORT      I2S_NUM_0
#define I2S_BCLK_PIN  4
#define I2S_LRCK_PIN  5
#define I2S_DIN_PIN   6

// We use a 1-element queue as a "Mailbox" to pass the latest 
// delta time without ever slowing down the reader task.
QueueHandle_t mailboxDeltaTime;

// ---------------------------------------------------------
// TASK 1: I2S Reader (Pinned to Core 1 - App Core)
// ---------------------------------------------------------
void task_i2s_reader(void *pvParameters) {
    // 1. Configure the I2S peripheral
    i2s_config_t i2s_config = {
        // CHANGED TO SLAVE
        .mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 1024,
        // APLL must be false for Slaves
        .use_apll = false 
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK_PIN,
        .ws_io_num = I2S_LRCK_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DIN_PIN
    };

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_start(I2S_PORT);

    uint32_t sample_data = 0;
    size_t bytes_read = 0;
    
    // Use the ESP-IDF high-resolution timer (microseconds)
    uint64_t last_time = esp_timer_get_time();

    while (true) {
        // Read 1 sample (4 bytes for 32-bit)
        // Note: Reading chunk by chunk (e.g., 1024 bytes) is highly recommended for real samplers
        i2s_read(I2S_PORT, &sample_data, 2, &bytes_read, portMAX_DELAY);

        uint64_t current_time = esp_timer_get_time();
        uint32_t delta_t_us = (uint32_t)(current_time - last_time);
        last_time = current_time;

        // Overwrite the mailbox with the latest delta time. 
        // This ensures the reader task NEVER blocks, achieving maximum loop speed.
        xQueueOverwrite(mailboxDeltaTime, &delta_t_us);
    }
}

// ---------------------------------------------------------
// TASK 2: Serial Plotter (Pinned to Core 0 - Pro Core)
// ---------------------------------------------------------
void task_serial_plotter(void *pvParameters) {
    uint32_t delta_to_print = 0;

    while (true) {
        // Peek or receive the latest value from the mailbox
        if (xQueueReceive(mailboxDeltaTime, &delta_to_print, portMAX_DELAY) == pdTRUE) {
            
            // Print to serial. (Serial operations are very slow compared to I2S)
            Serial.printf("Delta T: %lu us\n", delta_to_print);
            
            // Artificial delay to prevent Serial buffer overflow and visual spam.
            // Adjust this to control how often the plotter updates the screen.
            vTaskDelay(pdMS_TO_TICKS(100)); 
        }
    }
}

// ---------------------------------------------------------
// MAIN SETUP
// ---------------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial) { vTaskDelay(10); } // Wait for serial console to open
    
    Serial.println("Starting I2S Max Frequency Profiler...");

    // Create the 1-element mailbox queue
    mailboxDeltaTime = xQueueCreate(1, sizeof(uint32_t));

    // Core 0 handles the heavy lifting of String formatting and Serial UART
    xTaskCreatePinnedToCore(
        task_serial_plotter, 
        "SerialTask", 
        4096, 
        NULL, 
        1,       // Lower priority
        NULL, 
        0        // Pinned to Core 0
    );

    // Core 1 handles the high-speed I2S hardware reading
    xTaskCreatePinnedToCore(
        task_i2s_reader, 
        "I2STask", 
        4096, 
        NULL, 
        configMAX_PRIORITIES - 1, // Highest priority to preempt other tasks
        NULL, 
        1                         // Pinned to Core 1
    );
}

void loop() {
    // The Arduino loop runs automatically as a task on Core 1 with priority 1.
    // Since we are using custom tasks, we delete the Arduino loop task to free up memory.
    vTaskDelete(NULL); 
}