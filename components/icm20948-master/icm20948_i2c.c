#include "driver/i2c_master.h"
#include "icm20948.h"
#include "icm20948_i2c.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

icm20948_status_e icm20948_internal_write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
    icm20948_config_i2c_t *args = (icm20948_config_i2c_t*)user;
    
    uint8_t write_buf[len + 1];
    write_buf[0] = reg;
    memcpy(&write_buf[1], data, len);
    
    esp_err_t ret = i2c_master_transmit(args->i2c_handle, write_buf, len + 1, I2C_XFR_TIMEOUT_MS);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_INVALID_ARG)
            return ICM_20948_STAT_PARAM_ERR;
        else
            return ICM_20948_STAT_ERR; // ESP_ERR_TIMEOUT or other
    }
    
    return ICM_20948_STAT_OK;
}

icm20948_status_e icm20948_internal_read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
    icm20948_config_i2c_t *args = (icm20948_config_i2c_t*)user;
    
    esp_err_t ret = i2c_master_transmit(args->i2c_handle, &reg, 1, I2C_XFR_TIMEOUT_MS);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_INVALID_ARG)
            return ICM_20948_STAT_PARAM_ERR;
        else
            return ICM_20948_STAT_ERR; // ESP_ERR_TIMEOUT or other
    }
    
    ret = i2c_master_receive(args->i2c_handle, buff, len, I2C_XFR_TIMEOUT_MS);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_INVALID_ARG)
            return ICM_20948_STAT_PARAM_ERR;
        else
            return ICM_20948_STAT_ERR; // ESP_ERR_TIMEOUT or other
    }
    
    return ICM_20948_STAT_OK;
}

icm20948_serif_t default_serif = {
    icm20948_internal_write_i2c,
    icm20948_internal_read_i2c,
    NULL,
};

icm20948_status_e icm20948_init_i2c(i2c_master_bus_handle_t bus_handle, icm20948_config_i2c_t *args, icm20948_device_t *icm_dev)
{
    icm20948_status_e stat;
    icm20948_config_i2c_t *args_heap = malloc(sizeof(icm20948_config_i2c_t));
    *args_heap = *args; //Copy the struct
    
    esp_err_t ret = i2c_master_probe(bus_handle, args_heap->i2c_addr, I2C_XFR_TIMEOUT_MS);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NOT_FOUND)
            stat = ICM_20948_STAT_WRONG_ID;
        else
            stat = ICM_20948_STAT_ERR; // ESP_ERR_TIMEOUT or other
        free(args_heap);
        return stat;
    }

    const i2c_device_config_t i2c_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = args_heap->i2c_addr,
        .scl_speed_hz = args_heap->i2c_clock_speed,
    };
    
    if (args_heap->i2c_handle == NULL) {
        ret = i2c_master_bus_add_device(bus_handle, &i2c_dev_cfg, &args_heap->i2c_handle);
        if (ret != ESP_OK)
        {
            if (ret == ESP_ERR_INVALID_ARG)
                stat = ICM_20948_STAT_PARAM_ERR;
            else
                stat = ICM_20948_STAT_ERR; // ESP_ERR_NO_MEM or other
            goto err;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay to ensure the device is ready after adding to bus
    
    stat = icm20948_init_struct(icm_dev);
    if (stat != ICM_20948_STAT_OK)
        goto err;

    default_serif.user = (void *)args_heap;
    stat = icm20948_link_serif(icm_dev, &default_serif);
    if (stat != ICM_20948_STAT_OK)
        goto err;

    icm_dev->_dmp_firmware_available = false;
    icm_dev->_firmware_loaded = false;
    icm_dev->_last_bank = 255;
    icm_dev->_last_mems_bank = 255;
    icm_dev->_gyroSF = 0;
    icm_dev->_gyroSFpll = 0;
    icm_dev->_enabled_Android_0 = 0;
    icm_dev->_enabled_Android_1 = 0;
    icm_dev->_enabled_Android_intr_0 = 0;
    icm_dev->_enabled_Android_intr_1 = 0;

    return ICM_20948_STAT_OK;

    err:
        if (args_heap->i2c_handle)
            i2c_master_bus_rm_device(args_heap->i2c_handle);
        free(args_heap);
        return stat;
}