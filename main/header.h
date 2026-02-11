#ifndef HEADER_H
#define HEADER_H

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_err.h"
#include <esp_check.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "driver/i2c_master.h"
#include "driver/uart.h"

#include "sys/stat.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "unistd.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_littlefs.h"

#include "icm20948.h"
#include "icm20948_i2c.h"
#include "bmp390.h"
#include "vqf_c.h"
#include "minmea.h"

#define BUZZER_GPIO GPIO_NUM_38
#define LED_GPIO GPIO_NUM_15
#define ALED_GPIO GPIO_NUM_6
#define BUTTON_GPIO GPIO_NUM_0
#define VIN_ADC_UNIT ADC_UNIT_1 // ADC1 +
#define VIN_ADC_CHANNEL ADC_CHANNEL_4 // CH4 = GPIO05
#define DROGUE_GPIO GPIO_NUM_47
#define MAIN_GPIO GPIO_NUM_48
#define RBF_GPIO GPIO_NUM_4
#define GPS_RX GPIO_NUM_14
#define GPS_TX GPIO_NUM_21
#define MOSI GPIO_NUM_11
#define MISO GPIO_NUM_13
#define SCK GPIO_NUM_12
#define SS GPIO_NUM_10
#define LORA_RX GPIO_NUM_36
#define LORA_TX GPIO_NUM_35
#define LORA_AUX GPIO_NUM_37
#define LORA_M0 GPIO_NUM_40
#define LORA_M1 GPIO_NUM_39
#define I2C_SDA_IO GPIO_NUM_8
#define I2C_SCL_IO GPIO_NUM_9
#define GPS_UART_NUM UART_NUM_1
#define LORA_UART_NUM UART_NUM_2

#define I2C_SPEED 100000 // 100kHz

#define GPS_BAUDRATE 115200
#define LORA_BAUDRATE 115200

#define GPS_BUFF_SIZE 2048
#define SD_BUFFER_SIZE 4096
#define LITTLEFS_BUFFER_SIZE 512

#define LORA_RATE_MS 200 // 5Hz
#define CONFIG_E220_AUX_TIMEOUT_MS 1200 

#define MAX_LFS_FILES 32
#define MAX_SD_FILES 5
#define SD_UNIT_SIZE 16 * 1024
#define SD_TRANSF_SIZE 4000
#define SD_MOUNT "/sdcard"
#define MAX_FLASH_SIZE_USED 0.9 // Maximum percentage of flash to be used by littlefs
#define FILENAME_LENGTH 32

#define SD_QUEUE_SIZE 80
#define LITTLEFS_QUEUE_SIZE 80
#define LORA_QUEUE_SIZE 5

#define EVT_SD_DONE BIT0
#define EVT_LFS_DONE BIT1

#define BMP390_I2C_ADDRESS (0x77)
#define ICM20948_I2C_ADDRESS (0x68)

#define FUSION_SAMPLE_RATE_HZ 100

// Status flags
#define ARMED (1 << 0)
#define SAFE_MODE (1 << 1)
#define FLYING (1 << 2)
#define CUTOFF (1 << 3)
#define DROGUE_DEPLOYED (1 << 4)
#define MAIN_DEPLOYED (1 << 5)
#define LANDED (1 << 6)
#define LFS_FULL (1 << 7)

#define FLYING_THRESHOLD 15 // BMP altitude threshold to consider rocket flying in meters
#define CUTOFF_THRESHOLD 30  // Acceleration threshold to consider motor cutoff in g*10 (e.g. 30 means 3g)
#define LANDED_THRESHOLD 5  // Altitude threshold to consider rocket landed in meters
#define DROGUE_THRESHOLD 10 // Altitude drop threshold to deploy drogue in meters
#define MAIN_ALTITUDE 450 // Altitude above initial to deploy main in meters
#define KNOWN_ALTITUDE 715 // m, launch zone altitude

#define V_DIV_RATIO 0.015 // Voltage divider ratio {[(10k + 20k) / 20k] * 0.1}

// Scaling factors for sensors
#define GYRO_SCALE 1.0f/65.5f // ±500dps
#define ACC_SCALE 1.0f/2048.0f// ±16g
#define MAG_SCALE 0.15f
#define TEMP_SCALE 1.0f/333.87f
#define TEMP_OFFSET 14.0f // °C

// ICM20948 sample buffer
typedef struct
{
    int16_t temperature; // @SAVE (LSB)
    int16_t accel_x, accel_y, accel_z; // @SAVE (LSB)
    int16_t gyro_x, gyro_y, gyro_z; // @SAVE (LSB)
    int16_t mag_x, mag_y, mag_z; // @SAVE (LSB)
    float q1, q2, q3, q4; // @SEND (quaternions)
    uint8_t accel; // @SEND (|g| * 10, e.g. 159 for 15.9g)
    float initial_temperature; // @INTERNAL (°C)
} icm20948_sample_t;

// GPS sample buffer
typedef struct
{
    float latitude; // @SAVE + SEND (°)
    float longitude; // @SAVE + SEND (°)
    float altitude; // @SAVE + SEND (m)
    float vel_vertical; // @SAVE + SEND (m/s)
    float initial_altitude; // @INTERNAL (m)
    uint32_t utc_time; // @INTERNAL (HHMMSS)
} gps_sample_t;

// BMP390 sample buffer
typedef struct
{
    float altitude; // @SEND (m)
    float pressure; // @SAVE (Pa)
    float initial_altitude; // @INTERNAL (m)
} bmp390_sample_t;

// Kalman filter state vector elements
typedef struct
{
    float altitude; // @SEND (M)
    float vel_vertical; // @SEND (M/s)
    float apogee; // @SEND (M)
    float initial_altitude; // @INTERNAL (M)
} kalman_filter_state_t;

/** *
 * @brief Data structure to store sensor data.
 * @note Fields are grouped and ordered by size for memory efficiency.
 */
typedef struct
{
    uint32_t time; // @SAVE + SEND (Unix time in seconds, or GPS time in HHMMSS)

    icm20948_sample_t icm;
    gps_sample_t gps;
    bmp390_sample_t bmp;
    kalman_filter_state_t kf;
    
    uint8_t status; // @SAVE + SEND (Bitfields)
    uint8_t voltage; // @SAVE + SEND (V * 10, e.g. 33 for 3.3V)
} data_t;

/** *
 * @brief Data structure for saving to SD card and LittleFS.
 * @note Packed to avoid padding bytes.
 */
typedef struct __attribute__((packed))
{
    uint32_t time;
    float pressure;
    float latitude, longitude;
    float gps_altitude, gps_vel_vertical;
    int16_t temperature;
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t mag_x, mag_y, mag_z;
    uint8_t status;
    uint8_t voltage; // V * 10
} save_t;

/** *
 * @brief Data structure for sending to ground station.
 * @note Packed to avoid padding bytes.
 */
typedef struct __attribute__((packed))
{
    uint32_t time;
    float latitude, longitude;
    uint16_t kf_altitude, kf_apogee; // m * 10 (limited to 6553.5 m)
    int16_t kf_vel_vertical; // m/s * 10
    int16_t q1, q2, q3, q4; // quaternions * 10 000
    uint8_t accel; // |g| * 10
    uint8_t status;
    uint8_t voltage; // V * 10
} send_t;

extern data_t data_g;
extern icm20948_sample_t icm_sample_g;
extern gps_sample_t gps_sample_g;
extern bmp390_sample_t bmp_sample_g;
extern uint8_t battery_voltage_g;

// Data structure to store file counter
typedef struct
{
    int32_t file_numSD;
    int32_t file_numLFS;
    bool format;
} file_counter_t;

extern file_counter_t file_counter_g;

//Tasks
extern TaskHandle_t xTaskLora;
extern TaskHandle_t xTaskAcquire;

// Queues
extern QueueHandle_t xSDQueue;
extern QueueHandle_t xLittleFSQueue;
extern QueueHandle_t xLoraQueue;
extern TaskHandle_t xTaskDeploy;

// Mutexes
extern SemaphoreHandle_t xI2CMutex;

extern portMUX_TYPE xDATAMutex;
extern portMUX_TYPE xBMPMutex;
extern portMUX_TYPE xGPSMutex;
extern portMUX_TYPE xICMMutex;
extern portMUX_TYPE xADCMutex;

// Event group for NVS counter synchronization
extern EventGroupHandle_t xNVSCounterEvent;

extern i2c_master_bus_handle_t bus_handle;

void fusion_task(void *pvParameters);
void bmp_task(void *pvParameters);
void gps_task(void *pvParameters);
void task_acquire(void *pvParameters);
void task_deploy(void *pvParameters);
void task_adc(void *pvParameters);
void task_log(void *pvParameters);
void task_sd(void *pvParameters);
void task_littlefs(void *pvParameters);
void task_lora(void *pvParameters);
void save_nvs_counters(void *pvParameters);

#endif // HEADER_H