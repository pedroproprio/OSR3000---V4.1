#include "header.h"

static const char *TAG = "BMP390";
static bmp390_handle_t bmp_dev_hdl;

static const float medium_lapse_rate =  -153.84615f; // (K/Km)^-1, L: average lapse rate
static const float exponent = 0.1902665f; // -R*L/g*M (R: universal gas constant,
                                          // g: gravitational acceleration,
                                          // M: dry air molar mass)

static float sea_pressure = 0.0f;
static float pressure_offset = 0.0f;

esp_err_t bmp_init(void)
{
    // BMP390 Initialization
    bmp390_config_t bmp_cfg = I2C_BMP390_CONFIG_DEFAULT;
    bmp_cfg.i2c_address = BMP390_I2C_ADDRESS;
    bmp_cfg.i2c_clock_speed = I2C_SPEED;
    bmp_cfg.iir_filter = BMP390_IIR_FILTER_3;
    bmp390_handle_t bmp_dev_hdl;
    esp_err_t err = bmp390_init(bus_handle, &bmp_cfg, &bmp_dev_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

void bmp_task(void *pvParameters)
{
    data_t *data = (data_t *)pvParameters;
    while(true) {
        float temp_temperature = 0.0f, temp_pressure = 0.0f;
        xSemaphoreTake(xI2CMutex, portMAX_DELAY);
        esp_err_t err = bmp390_get_measurements(bmp_dev_hdl, &temp_temperature, &temp_pressure);
        xSemaphoreGive(xI2CMutex);
        if(err != ESP_OK)
            ESP_LOGE(TAG, "bmp390 device read failed (%s)", esp_err_to_name(err));
        if (sea_pressure == 0)
            sea_pressure = get_sea_pressure(temp_pressure);
        float alt = get_altitude_from_pressure(temp_pressure);
        if (bmp_initial_alt == 0)
            bmp_initial_alt = bmp_get_initial_alt(alt);
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        data->pressure = temp_pressure;
        data->bmp_altitude = alt;
        xSemaphoreGive(xDataMutex);
        alt = bar_mahalanobis(alt);
        if (STATUS & ARMED)
            altitude_update_bar((alt-bmp_initial_alt));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    bmp390_delete(bmp_dev_hdl);
    vTaskDelete(NULL);
}

float get_altitude_from_pressure(const float pressure)
{
    float alt;
    if (initial_temp == 0)
        alt = (powf(pressure/101325, exponent)-1)*298*medium_lapse_rate; // stantard sea pressure + fixed 25°C 
    alt = (powf(pressure/sea_pressure, exponent)-1)*initial_temp*medium_lapse_rate;
    return alt;
}

float get_sea_pressure(const float pressure)
{
    if (initial_temp == 0)
        return 0.0f;
    static uint8_t i = 0;
    static float sum = 0;
    if (i < 20) { // get mean of 20 samples
        i++;
        sum += pressure;
        return 0.0f;
    }
    float mean = sum/20.0f;
    pressure_offset = mean - KNOWN_PRESSURE;
    sea_pressure = mean/powf((KNOWN_ALTITUDE/(initial_temp*medium_lapse_rate)) + 1, 1.0f/exponent);
    return sea_pressure;
}

float bmp_get_initial_alt(const float alt)
{
    static uint8_t i = 0;
    static float sum = 0;
    if (i < 100) { // get mean of 100 samples
        i++;
        sum += alt;
        return 0.0f;
    }
    float mean = sum/100.0f;
    return mean;
}