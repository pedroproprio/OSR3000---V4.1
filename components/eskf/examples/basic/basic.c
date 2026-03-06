// Include eskf header
#include "eskf.h"
// Additional headers
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void app_main(void) {
    // Configure the filter parameters
    eskf_t eskf;

    eskf_var_t var = {
        .acc = 0.25f, // acceleromter noise variance (m/s²)²
        .bar = 1.65f, // barometer noise variance (m)²
        .gps_h = 4.0f, // GPS altitude noise variance (m)²
        .gps_vz = 0.04f, // GPS vertical velocity noise variance (m/s)²
        .ba = 1e-3f, // accelerometer bias random walk variance (m/s²)²/s
        .bb = 1e-3f, // barometer bias random walk variance (m)²/s
        .θe = 1e-6f, // angular error random walk variance (rad)²/s
    };

    eskf_config_t cfg = {
        .var = var,
        .dt = 0.01f, // time step (s) of prediction updates
        .g = 9.81f, // magnitude of acceleration due to gravity (m/s²)
        .igt = 3.0f, // innovation gate threshold for outlier rejection (if r²/S > igt², updates are rejected). e.g., igt = 3 means 3σ gate. igt = 0 means no gating.
        .idle_samples = 100, // Number of consecutive low-acceleration samples to consider as idle (ZUPT updates)
    };

    eskf.cfg = cfg;

    // Main loop
    while (true) {
        // Read sensor data
        // ...

        // New accelerometer measurement
        float az = 0.1f; // vertical acceleration (m/s²)
        eskf_predict(&eskf, az, 1.0f);

        // New barometer measurement
        float h_bar = 2.0f; // altitude (m)
        eskf_update_bar(&eskf, h_bar, 1.0f);

        // New GPS measurements
        float h_gps = 5.0f; // altitude (m)
        float vz = 0.1f; // vertical velocity (m/s)
        eskf_update_gps(&eskf, h_gps, vz, 1.0f);

        ESP_LOGI("ESKF", "Altitude: %.2f m", eskf.x.h);
        ESP_LOGI("ESKF", "Vertical velocity: %.2f m/s", eskf.x.vz);
        ESP_LOGI("ESKF", "Apogee: %.2f m", eskf.x.apogee);

        vTaskDelay(pdMS_TO_TICKS(100));
    }

}