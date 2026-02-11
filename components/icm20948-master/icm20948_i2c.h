#ifndef _ICM_20948_I2C_H_
#define _ICM_20948_I2C_H_

#include "driver/i2c_master.h"

#define I2C_XFR_TIMEOUT_MS 500

/**
 * @brief Estrutura de configuração I2C do ICM20948.
 */
typedef struct {
    i2c_master_dev_handle_t i2c_handle;
    uint16_t i2c_addr;
    uint32_t i2c_clock_speed;
} icm20948_config_i2c_t;

/**
 * @brief Initializes an ICM20948 device onto the master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] icm20948_config ICM20948 I2C configuration.
 * @param[out] icm20948_handle ICM20948 device configuration.
 * @return esp_err_t ESP_OK on success.
 */
icm20948_status_e icm20948_init_i2c(i2c_master_bus_handle_t bus_handle, icm20948_config_i2c_t *args, icm20948_device_t *icm_dev);

/* these functions are exposed in order to make a custom setup of a serif_t possible */
icm20948_status_e icm20948_internal_write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user);
icm20948_status_e icm20948_internal_read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user);

#endif