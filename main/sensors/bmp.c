#include "header.h"

static const float medium_lapse_rate = -0.0065f; // -K/Km, L: average lapse rate
static const float exponent = -0.1902665f; // -R*L/g*M (R: universal gas constant,
                                                     // g: gravitational acceleration,
                                                     // M: dry air molar mass)

static const float standart_sea_pressure = 101325.0f; // Pa
static const float standart_initial_temp = 298.0f; // K (25 C)

static float sea_pressure = 0.0f;
static float initial_temp = 0.0f;

static float get_altitude_from_pressure(const float pressure)
{
    float check_sea_pressure = (sea_pressure == 0) ? standart_sea_pressure : sea_pressure;
    float check_initial_temp = (initial_temp == 0) ? standart_initial_temp : initial_temp;

    float alt = (1 - powf(pressure / check_sea_pressure, exponent)) * check_initial_temp / medium_lapse_rate;
    return alt;
}

// Calculate sea level pressure based on the known altitude and the mean pressure at that altitude
static float get_sea_pressure(const float pressure)
{
    static uint8_t i = 0;
    static float sum = 0;
    
    // get mean of 20 samples
    if (i < 20)
    { 
        i++;
        sum += pressure;
        return 0.0f;
    }

    if (initial_temp == 0)
        return 0.0f;
    
    float mean = sum / i;
    sea_pressure = mean / powf((1 - KNOWN_ALTITUDE * medium_lapse_rate / initial_temp), 1.0f / exponent);
    return sea_pressure;
}

static float bmp_get_initial_alt(const float alt)
{
    if (sea_pressure == 0) // initial altitude would be less reliable
        return 0.0f;

    static uint8_t i = 0;
    static float sum = 0;
    
    // get mean of 20 samples
    if (i < 20)
    {
        i++;
        sum += alt;
        return 0.0f;
    }

    float mean = sum / i;
    return mean;
}

static void bmp_init(bmp390_handle_t *bmp_hdl)
{
    // BMP390 Initialization
    bmp390_config_t bmp_cfg = I2C_BMP390_CONFIG_DEFAULT;
    bmp_cfg.i2c_address = BMP390_I2C_ADDRESS;
    bmp_cfg.i2c_clock_speed = I2C_SPEED;
    bmp_cfg.iir_filter = BMP390_IIR_FILTER_3;
    
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    ESP_ERROR_CHECK(bmp390_init(bus_handle, &bmp_cfg, bmp_hdl));
    xSemaphoreGive(xI2CMutex);
    ESP_LOGI("BMP390", "BMP390 initialized");
}

void bmp_task(void *pvParameters)
{
    bmp390_handle_t bmp_hdl;
    bmp_init(&bmp_hdl);
    bmp390_sample_t sample;
    static float initial_alt = 0.0f;
    
    while(true)
    {
        float temp_not_used;

        xSemaphoreTake(xI2CMutex, portMAX_DELAY);
        esp_err_t err = bmp390_get_measurements(bmp_hdl, &temp_not_used, &sample.pressure);
        xSemaphoreGive(xI2CMutex);
        if (err != ESP_OK)
        {
            ESP_LOGE("BMP390", "get measurements failed: %s", esp_err_to_name(err));
            continue;
        }

        if (initial_temp == 0)
        {
            icm20948_sample_t icm_sample;
            portENTER_CRITICAL(&xICMMutex);
            icm_sample = icm_sample_g;
            portEXIT_CRITICAL(&xICMMutex);
            initial_temp = icm_sample.initial_temperature;
        }

        // get sea level pressure
        if (sea_pressure == 0)
            sea_pressure = get_sea_pressure(sample.pressure);

        // get altitude
        float alt = get_altitude_from_pressure(sample.pressure);

        if (initial_alt == 0)
            initial_alt = bmp_get_initial_alt(alt); // in this logic, will be ±KNOWN_ALTITUDE

        // update global BMP sample
        portENTER_CRITICAL(&xBMPMutex);
        bmp_sample_g.pressure = sample.pressure;
        bmp_sample_g.altitude = alt;
        bmp_sample_g.initial_altitude = initial_alt;
        portEXIT_CRITICAL(&xBMPMutex);

        xTaskNotifyGive(xTaskAcquire); // Notify acquire task that new data is available

        portENTER_CRITICAL(&xDATAMutex);
        bool landed = (data_g.status & LANDED);
        portEXIT_CRITICAL(&xDATAMutex);
        if (landed)
            break;

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    bmp390_delete(bmp_hdl);
    xSemaphoreGive(xI2CMutex);
    vTaskDelete(NULL);
}