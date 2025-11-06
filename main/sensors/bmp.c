#include "header.h"

static const char *TAG = "BMP390";
static bmp390_handle_t bmp_dev_hdl;

static const float medium_lapse_rate =  153.84615f; // (K/Km)^-1, L: average lapse rate
static const float exponent = 0.1902665f; // R*L/g*M (R: universal gas constant,
                                          // g: gravitational acceleration,
                                          // M: dry air molar mass)

static float sea_pressure = 0.0f;
static float pressure_offset = 0.0f;

esp_err_t bmp_init(void)
{
    i2c_master_dev_handle_t dev_handle = NULL;
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bmp_config, &dev_handle));
    // BMP390 Initialization
    bmp390_config_t dev_cfg = I2C_BMP390_CONFIG_DEFAULT;
    dev_cfg.i2c_address = BMP390_I2C_ADDRESS;
    esp_err_t err;
    err = bmp390_init(bus_handle, &dev_cfg, &bmp_dev_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(err));
        return err;
    }
    err = bmp390_set_iir_filter(bmp_dev_hdl, BMP390_IIR_FILTER_3);
    if (err != ESP_OK) ESP_LOGE(TAG, "set IIR filter failed: %s", esp_err_to_name(err));
    return ESP_OK;
}

void bmp_task(data_t *data)
{
    float temp_temperature = 0.0f, temp_pressure = 0.0f;

    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    esp_err_t err = bmp390_get_measurements(bmp_dev_hdl, &temp_temperature, &temp_pressure);
    xSemaphoreGive(xI2CMutex);

    if(err != ESP_OK) 
        ESP_LOGE(TAG, "bmp390 device read failed (%s)", esp_err_to_name(err));
    if (sea_pressure == 0) sea_pressure = get_sea_pressure(temp_pressure);
    float alt = get_altitude_from_pressure(temp_pressure);
    if (bmp_initial_alt == 0) bmp_initial_alt = bmp_get_initial_alt(alt);
    data->pressure = temp_pressure;
    data->temperature = temp_temperature;
    alt = bar_mahalanobis(alt);
    data->bmp_altitude = alt;
    if (STATUS & ARMED)
        altitude_update_bar((alt-bmp_initial_alt));
}

float get_altitude_from_pressure(const float pressure)
{
    float alt;
    if (initial_temp == 0) 
        alt = (1-powf(pressure/sea_pressure, exponent))*298*medium_lapse_rate; // fixed 25°C 
    else 
        alt = (1-powf(pressure/sea_pressure, exponent))*initial_temp*medium_lapse_rate;
    if (sea_pressure == 0) 
        alt = (1- powf(pressure/101325, exponent))*298*medium_lapse_rate;
    return alt;
}

float get_sea_pressure(const float pressure)
{
    if (initial_temp == 0) return 0.0f;
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