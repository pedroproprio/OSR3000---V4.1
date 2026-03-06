#include "header.h"

#define CALIBRATION_SAMPLES 20

static const char *TAG = "BMP390";

static const float medium_lapse_rate = -0.0065f; // -K/m, L

// R: universal gas constant, g: gravitational acceleration, M: dry air molar mass
static const float exponent = -0.1902665f;      // -R*L/g*M
static const float inv_exponent = -5.25578596f; // -g*M/R*L

static float sea_pressure = 0.0f;
static float sea_temp = 0.0f;

static float get_altitude_from_pressure(float pressure)
{
    float alt = (sea_temp / medium_lapse_rate) * (1 - powf(pressure / sea_pressure, exponent));
    return alt;
}

static void bmp_init(bmp390_handle_t *bmp_hdl)
{
    // BMP390 struct setup
    bmp390_config_t bmp_cfg = {
        .i2c_address = BMP390_I2C_ADDRESS,
        .i2c_clock_speed = I2C_SPEED,
        .power_mode = BMP390_POWER_MODE_NORMAL,
        .iir_filter = BMP390_IIR_FILTER_1,
        .pressure_oversampling = BMP390_PRESSURE_OVERSAMPLING_4X,
        .temperature_oversampling   = BMP390_TEMPERATURE_OVERSAMPLING_2X,
        .output_data_rate = BMP390_ODR_20MS,
    };
    // bmp390_config_t bmp_cfg = I2C_BMP390_CONFIG_DEFAULT;
    // bmp_cfg.i2c_address = BMP390_I2C_ADDRESS;
    // bmp_cfg.i2c_clock_speed = I2C_SPEED;
    // bmp_cfg.iir_filter = BMP390_IIR_FILTER_1;

    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    ESP_ERROR_CHECK(bmp390_init(bus_handle, &bmp_cfg, bmp_hdl));
    xSemaphoreGive(xI2CMutex);

    ESP_LOGI(TAG, "BMP390 initialized");
    vTaskDelay(pdMS_TO_TICKS(100)); // wait for parameter changes ((1 + IIR_COEFF) * T_OS + 2.5 ms ?)

    // Configuration registers
    bmp390_power_control_register_t     power_ctrl_reg;
    bmp390_configuration_register_t     config_reg;
    bmp390_oversampling_register_t      oversampling_reg;
    bmp390_output_data_rate_register_t  output_data_rate_reg;
    bmp390_interrupt_control_register_t interrupt_ctrl_reg;

    // Set power mode manually
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    ESP_ERROR_CHECK(bmp390_set_power_mode(*bmp_hdl, bmp_cfg.power_mode));
    // Attempt to read registers
    bmp390_get_configuration_register(*bmp_hdl, &config_reg);
    bmp390_get_oversampling_register(*bmp_hdl, &oversampling_reg);
    bmp390_get_power_control_register(*bmp_hdl, &power_ctrl_reg);
    bmp390_get_output_data_rate_register(*bmp_hdl, &output_data_rate_reg);
    bmp390_get_interrupt_control_register(*bmp_hdl, &interrupt_ctrl_reg);
    xSemaphoreGive(xI2CMutex);

    ESP_LOGD(TAG, "Configuration (0x%02x): %s", config_reg.reg,           uint8_to_binary(config_reg.reg));
    ESP_LOGD(TAG, "Oversampling  (0x%02x): %s", oversampling_reg.reg,     uint8_to_binary(oversampling_reg.reg));
    ESP_LOGD(TAG, "Data Rate     (0x%02x): %s", output_data_rate_reg.reg, uint8_to_binary(output_data_rate_reg.reg));
    ESP_LOGD(TAG, "Power Control (0x%02x): %s", power_ctrl_reg.reg,       uint8_to_binary(power_ctrl_reg.reg));
    ESP_LOGD(TAG, "Int Control   (0x%02x): %s", interrupt_ctrl_reg.reg,   uint8_to_binary(interrupt_ctrl_reg.reg));
}

void bmp_task(void *pvParameters)
{
    bmp390_handle_t bmp_hdl;
    bmp_init(&bmp_hdl);

    bmp390_sample_t sample;

    // Calibration samples 
    float pressure_sum = 0.0f;
    uint8_t pressure_samples = 0;

    // Get sea level temperature using KNOWN_ALTITUDE
    sea_temp = KNOWN_TEMPERATURE + 273.15f + (medium_lapse_rate * KNOWN_ALTITUDE);
    ESP_LOGI(TAG, "Sea level temperature calibrated: %.2f°C", sea_temp - 273.15f);

    while (true)
    {
        float temperature;

        xSemaphoreTake(xI2CMutex, portMAX_DELAY);
        esp_err_t err = bmp390_get_measurements(bmp_hdl, &temperature, &sample.pressure);
        xSemaphoreGive(xI2CMutex);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Get measurements failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(BMP_SAMPLE_RATE_MS));
            continue;
        }

        // Calibrate sea level pressure
        if (pressure_samples < CALIBRATION_SAMPLES)
        {
            pressure_sum += sample.pressure;
            pressure_samples++;

            if (pressure_samples == CALIBRATION_SAMPLES)
            {
                float mean_pressure = pressure_sum / pressure_samples;
                sea_pressure = mean_pressure / powf((1 - KNOWN_ALTITUDE * medium_lapse_rate / sea_temp), inv_exponent);
                ESP_LOGI(TAG, "Sea level pressure calibrated: %.2f Pa", sea_pressure);
            }

            vTaskDelay(pdMS_TO_TICKS(BMP_SAMPLE_RATE_MS));
            continue;
        }

        // Get altitude
        float alt = get_altitude_from_pressure(sample.pressure);

        // Update global BMP sample
        portENTER_CRITICAL(&xBMPMutex);
        bmp_sample_g.pressure = sample.pressure;
        bmp_sample_g.altitude = alt - KNOWN_ALTITUDE;
        portEXIT_CRITICAL(&xBMPMutex);

        // Notify acquire task that new data is available
        xTaskNotify(xTaskAcquire, BMP_BIT, eSetBits);

        portENTER_CRITICAL(&xDATAMutex);
        bool landed = (data_g.status & LANDED);
        portEXIT_CRITICAL(&xDATAMutex);

        if (landed)
            break;

        vTaskDelay(pdMS_TO_TICKS(BMP_SAMPLE_RATE_MS));
    }

    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    bmp390_delete(bmp_hdl);
    xSemaphoreGive(xI2CMutex);
    vTaskDelete(NULL);
}