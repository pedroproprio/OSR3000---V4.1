#include "header.h"

i2c_master_bus_handle_t bus_handle = NULL;

TaskHandle_t xTaskLora = NULL;
TaskHandle_t xTaskAcquire = NULL;
TaskHandle_t xTaskDeploy = NULL;

// Queues
QueueHandle_t xSDQueue = NULL;
QueueHandle_t xLittleFSQueue = NULL;
QueueHandle_t xLoraQueue = NULL;

// Mutexes
SemaphoreHandle_t xI2CMutex = NULL;

portMUX_TYPE xDATAMutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE xBMPMutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE xGPSMutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE xICMMutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE xADCMutex = portMUX_INITIALIZER_UNLOCKED;

EventGroupHandle_t xNVSCounterEvent = NULL;

data_t data_g = {0};
bmp390_sample_t bmp_sample_g = {0};
gps_sample_t gps_sample_g = {0};
icm20948_sample_t icm_sample_g = {0};
uint8_t battery_voltage_g = 0;
file_counter_t file_counter_g = {0};