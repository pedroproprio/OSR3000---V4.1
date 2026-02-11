#include "header.h"

#define GYRO_SCALE_RAD GYRO_SCALE * 3.14159265359f / 180.0f // Convert from °/s to rad/s
#define ACC_SCALE_MS2 ACC_SCALE * 9.80665f // Convert from g to m/s²

static const char *TAG = "ICM20948";

// Gravity vector in sensor frame calculated from quaternions
static void getGravityVector(float qw, float qx, float qy, float qz, float g[3])
{
    g[0] = 2.0f * (qx * qz - qw * qy);
    g[1] = 2.0f * (qy * qz + qw * qx);
    g[2] = 2.0f * (qw * qw - 0.5f + qz * qz);
}

static void icm_init(icm20948_device_t *icm_dev)
{
    icm20948_config_i2c_t icm_config = {
        .i2c_addr = ICM20948_I2C_ADDRESS,
        .i2c_clock_speed = I2C_SPEED,
    };
    
    /* setup icm20948 device */
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    ESP_ERROR_CHECK(icm20948_init_i2c(bus_handle, &icm_config, icm_dev));
    xSemaphoreGive(xI2CMutex);
    ESP_LOGI(TAG, "ICM20948 initialized");

    /* check ID */
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    while (icm20948_check_id(icm_dev) != ICM_20948_STAT_OK)
    {
        ESP_LOGE(TAG, "check id failed");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    xSemaphoreGive(xI2CMutex);
    ESP_LOGI(TAG, "check id passed");

    // Here we are doing a SW reset to make sure the device starts in a known state
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    icm20948_status_e stat = icm20948_sw_reset(icm_dev);
    xSemaphoreGive(xI2CMutex);
    if (stat != ICM_20948_STAT_OK)
        ESP_LOGE(TAG, "reset failed: %d", stat);
    vTaskDelay(pdMS_TO_TICKS(250));

    // Now wake the sensor up
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    stat = icm20948_sleep(icm_dev, false);
    xSemaphoreGive(xI2CMutex);
    if (stat != ICM_20948_STAT_OK)
        ESP_LOGE(TAG, "wake up failed: %d", stat);
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    stat = icm20948_low_power(icm_dev, false);
    xSemaphoreGive(xI2CMutex);
    if (stat != ICM_20948_STAT_OK)
        ESP_LOGE(TAG, "normal power failed: %d", stat);

    icm20948_internal_sensor_id_bm sensors = (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);

    // Set Gyro and Accelerometer to a particular sample mode
    // options: SAMPLE_MODE_CONTINUOUS; SAMPLE_MODE_CYCLED
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    stat = icm20948_set_sample_mode(icm_dev, sensors, SAMPLE_MODE_CONTINUOUS); 
    xSemaphoreGive(xI2CMutex);
    if (stat != ICM_20948_STAT_OK)
        ESP_LOGE(TAG, "set sample mode failed: %d", stat);

    // Set up sensors sample rate
    icm20948_smplrt_t smplrt;
    smplrt.a = 10;
    smplrt.g = 10;
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    stat = icm20948_set_sample_rate(icm_dev, sensors, smplrt);
    xSemaphoreGive(xI2CMutex);
    if (stat != ICM_20948_STAT_OK)
        ESP_LOGE(TAG, "set sample rate failed: %d", stat);

    // Set full scale ranges for both acc and gyr
    icm20948_fss_t myfss;
    myfss.a = GPM_16;
    myfss.g = DPS_500;
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    stat = icm20948_set_full_scale(icm_dev, sensors, myfss);
    xSemaphoreGive(xI2CMutex);
    if (stat != ICM_20948_STAT_OK)
        ESP_LOGE(TAG, "set scale failed: %d", stat);

    // Set up Digital Low Pass Filter configuration
    icm20948_dlpcfg_t myDLPcfg;
    myDLPcfg.a = ACC_D111BW4_N136BW;
    myDLPcfg.g = GYR_D119BW5_B154BW3;
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    stat = icm20948_set_dlpf_cfg(icm_dev, sensors, myDLPcfg);
    xSemaphoreGive(xI2CMutex);
    if (stat != ICM_20948_STAT_OK)
        ESP_LOGE(TAG, "set DLPF failed: %d", stat);

    // Choose whether or not to use DLPF
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    stat = icm20948_enable_dlpf(icm_dev, ICM_20948_INTERNAL_ACC, true);
    xSemaphoreGive(xI2CMutex);
    if (stat != ICM_20948_STAT_OK)
        ESP_LOGE(TAG, "accel DLPF failed: %d", stat);
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    stat = icm20948_enable_dlpf(icm_dev, ICM_20948_INTERNAL_GYR, true);
    xSemaphoreGive(xI2CMutex);
    if (stat != ICM_20948_STAT_OK)
        ESP_LOGE(TAG, "gyro DLPF failed: %d", stat);

    // Initialize magnetometer
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    stat = icm20948_init_magnetometer(icm_dev);
    xSemaphoreGive(xI2CMutex);
    if (stat != ICM_20948_STAT_OK)
        ESP_LOGE(TAG, "mag init failed: %d", stat);
}

void fusion_task(void *pvParameters)
{
    icm20948_device_t icm_dev;
    icm_init(&icm_dev);

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / FUSION_SAMPLE_RATE_HZ);

    vqf_params_t params;
    vqf_params_init(&params);
    vqf_handle_t* vqf = vqf_init_custom(&params, 1.0f / FUSION_SAMPLE_RATE_HZ, -1, 1.0f * 32 / 1100.0f); // gyrTs, accTs, magTs

    icm20948_sample_t icm;
    static float initial_temp = 0.0f;

    xLastWakeTime = xTaskGetTickCount();
    while(true)
	{
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
        icm20948_agmt_t agmt;
        xSemaphoreTake(xI2CMutex, portMAX_DELAY);
        icm20948_status_e stat = icm20948_get_agmt(&icm_dev, &agmt);
        xSemaphoreGive(xI2CMutex);
		if (stat == ICM_20948_STAT_OK)
        {
            if(initial_temp == 0)
                initial_temp = (agmt.tmp.val*TEMP_SCALE + TEMP_OFFSET) + 273;

            float gyr[3] = {agmt.gyr.axes.x * GYRO_SCALE_RAD, agmt.gyr.axes.y * GYRO_SCALE_RAD, agmt.gyr.axes.z * GYRO_SCALE_RAD};
            float acc[3] = {agmt.acc.axes.x * ACC_SCALE_MS2, agmt.acc.axes.y * ACC_SCALE_MS2, agmt.acc.axes.z * ACC_SCALE_MS2};
            float mag[3] = {agmt.mag.axes.x * MAG_SCALE, agmt.mag.axes.y * MAG_SCALE, agmt.mag.axes.z * MAG_SCALE};
            vqf_update(vqf, gyr, acc, mag);
            
            float q[4];
            vqf_get_quat9D(vqf, q);
            
            icm.q1 = q[0];
            icm.q2 = q[1];
            icm.q3 = q[2];
            icm.q4 = q[3];
            
            // Update raw sensor data
            icm.accel_x = agmt.acc.axes.x;
            icm.accel_y = agmt.acc.axes.y;
            icm.accel_z = agmt.acc.axes.z;
            
            icm.gyro_x = agmt.gyr.axes.x;
            icm.gyro_y = agmt.gyr.axes.y;
            icm.gyro_z = agmt.gyr.axes.z;
            
            icm.mag_x = agmt.mag.axes.x;
            icm.mag_y = agmt.mag.axes.y;
            icm.mag_z = agmt.mag.axes.z;
            
            float g[3];
            getGravityVector(q[0], q[1], q[2], q[3], g);
            float _acc[3] = {agmt.acc.axes.x * ACC_SCALE - g[0], agmt.acc.axes.y * ACC_SCALE - g[1], agmt.acc.axes.z * ACC_SCALE - g[2]};
            float accel = sqrtf(_acc[0]*_acc[0] + _acc[1]*_acc[1] + _acc[2]*_acc[2]);
            icm.accel = (uint8_t) lroundf(accel * 10.0f);
            icm.temperature = agmt.tmp.val;
            icm.initial_temperature = initial_temp;

            // update global ICM sample
            portENTER_CRITICAL(&xICMMutex);
            icm_sample_g = icm;
            portEXIT_CRITICAL(&xICMMutex);

            xTaskNotifyGive(xTaskAcquire); // Notify acquire task that new data is available
		} else
			ESP_LOGE(TAG, "sensor reading failed");

        portENTER_CRITICAL(&xDATAMutex);
        bool landed = (data_g.status & LANDED);
        portEXIT_CRITICAL(&xDATAMutex);
        if (landed)
            break;
    }

    vqf_delete(vqf);
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    icm20948_sleep(&icm_dev, true);
    xSemaphoreGive(xI2CMutex);
    vTaskDelete(NULL);
}