#ifndef HEADER_H
#define HEADER_H

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_err.h"

#include "icm20948.h"
#include "icm20948_i2c.h"
#include "bmp390.h"
#include "Fusion.h"
#include "minmea.h"
#include "altitude.h"

#include "driver/i2c_master.h"
#include "driver/uart.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "sys/stat.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "unistd.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_littlefs.h"

#define DROGUE_THRESHOLD 10
#define MAIN_ALTITUDE 450
#define KNOWN_PRESSURE 0 // pascal, given by local weather station
#define KNOWN_ALTITUDE 0 // m, given by local weather station

#define BUZZER_GPIO GPIO_NUM_38
#define LED_GPIO GPIO_NUM_15
#define ALED_GPIO GPIO_NUM_6
#define BUTTON_GPIO GPIO_NUM_0
#define DROGUE_GPIO GPIO_NUM_47
#define MAIN_GPIO GPIO_NUM_48
#define RBF_GPIO GPIO_NUM_4
#define GPS_RX GPIO_NUM_21
#define MOSI GPIO_NUM_11
#define MISO GPIO_NUM_13
#define SCK GPIO_NUM_12
#define SS GPIO_NUM_10
#define LORA_RX GPIO_NUM_35
#define LORA_TX GPIO_NUM_36
#define LORA_AUX GPIO_NUM_37
#define LORA_M0 GPIO_NUM_40
#define LORA_M1 GPIO_NUM_39
#define I2C_SDA_IO GPIO_NUM_8
#define I2C_SCL_IO GPIO_NUM_9

#define I2C_SPEED 40000

#define GPS_BAUDRATE 115200
#define LORA_BAUDRATE 115200
#define GPS_BUFF_SIZE 1024
#define LORA_BUFF_SIZE 255
#define SD_BUFFER_SIZE 1024
#define LITTLEFS_BUFFER_SIZE 1024
#define LORA_TX_RETRIES 2
#define CONFIG_E220_TX_DONE_TIMEOUT_MS 1000 
#define CONFIG_E220_AUX_TIMEOUT_MS 3000 
#define LORA_UART_NUM UART_NUM_2
#define MAX_LFS_FILES 32
#define MAX_SD_FILES 5
#define SD_UNIT_SIZE 16 * 1024
#define SD_TRANSF_SIZE 4000
#define SD_MOUNT "/sdcard"
#define MAX_USED 0.8 // Maximum percentage of flash to be used by littlefs
#define FILENAME_LENGTH 32

#define BMP390_I2C_ADDRESS (0x76)

#define FUSION_SAMPLE_RATE 100

// Status flags
#define ARMED (1 << 0)
#define SAFE_MODE (1 << 1)
#define FLYING (1 << 2)
#define CUTOFF (1 << 3)

#define DROGUE_DEPLOYED (1 << 4)
#define MAIN_DEPLOYED (1 << 5)
#define LANDED (1 << 6)
#define LFS_FULL (1 << 7)

#define G 9.80665

#define FLYING_THRESHOLD 15 // BMP altitude threshold to consider rocket flying in meters
#define CUTOFF_THRESHOLD 2  // Acceleration threshold to consider motor cutoff in g's
#define LANDED_THRESHOLD 2  // Altitude threshold to consider rocket landed in meters

#define SAVE_SIZE 47 // bytes according to struct defined on queue
#define SEND_SIZE 52

#define R1 10000.0f // Resistor connected to battery positive terminal
#define R2 20000.0f // Resistor connected to ground

// Data structure to store sensor data
typedef struct // size = 77 bytes
{
    // SAVE - 24 bytes:
    
    float pressure;
    uint16_t temperature; // °C * 100
    
    int16_t accel_x; // LSB
    int16_t accel_y;
    int16_t accel_z;
    
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    
    
    // SAVE + SEND - 24 bytes:

    uint32_t time;
    uint16_t status;
    uint16_t voltage; // V * 100

    float latitude;
    float longitude;
    float gps_altitude;
    float gps_vel_vertical;

    // SEND - 29 bytes:

    float bmp_altitude;
    uint8_t accel; // accel module in g's * 10 (limited to ±25.5g)
    
    // quaternions
    float orientation_q1;
    float orientation_q2;
    float orientation_q3;
    float orientation_q4;
    
    // kalman filter state vector elements
    float kf_altitude;
    float kf_vel_vertical;
} data_t;

extern uint32_t utc_time;
extern float initial_temp; // kelvin
extern float bmp_initial_alt;
extern float gps_initial_alt;

// Data structure to store file counter
typedef struct
{
    uint32_t file_num;
    uint32_t format;
} file_counter_t;

//Tasks
extern TaskHandle_t xTaskLora;

// Queues
extern QueueHandle_t xAltQueue;
extern QueueHandle_t xLittleFSQueue;
extern QueueHandle_t xSDQueue;
extern QueueHandle_t xLoraQueue;

// Mutexes
extern SemaphoreHandle_t xGPSMutex;
extern SemaphoreHandle_t xStatusMutex;
extern SemaphoreHandle_t xI2CMutex;

// Status
extern uint16_t STATUS;

extern i2c_master_bus_handle_t bus_handle;
extern i2c_master_bus_config_t bus_config;
extern i2c_device_config_t dev_cfg;
extern i2c_device_config_t bmp_config;
extern uart_config_t uart_config;
extern altitude_config_t kf_config;

esp_err_t icm_init(void);
esp_err_t bmp_init(void);
void fusion_task(data_t *data, FusionOffset* offset, FusionAhrs* ahrs, icm20948_agmt_t* agmt);
void bmp_task(data_t *data);
float get_altitude_from_pressure(const float pressure);
float get_sea_pressure(const float pressure);
float bmp_get_initial_alt(const float alt);
float gps_get_initial_alt(const float alt);
void gps_task(data_t *data, uint8_t buffer[GPS_BUFF_SIZE]);
bool ubx_parse(uint8_t *buffer, int len, data_t *data);
void ubx_calculate_checksum(uint8_t *buffer, uint16_t length, uint8_t *ck_a, uint8_t *ck_b);

void init_adc(adc_oneshot_unit_handle_t *adc_handle, adc_cali_handle_t *cali_handle);
void acquire_voltage(data_t *data, adc_oneshot_unit_handle_t *adc_handle, adc_cali_handle_t *cali_handle);
void status_checks(data_t *data);
void send_queues(data_t *data);
void task_acquire(void *pvParameters);
void task_sd(void *pvParameters);
void task_littlefs(void *pvParameters);
void task_lora(void *pvParameters);
void save_struct(const data_t *src, data_t *dst);
void send_struct(const data_t *src, data_t *dst);

#endif