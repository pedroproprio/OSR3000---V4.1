#include "header.h"

static const char *TAG_ICM = "ICM20948";
static const char *TAG_FUSION = "FUSION";

static const float gyro_scale = 1.0f/65.5f;
static const float acc_scale = 1.0f/2048.0f;
static const float mag_scale = 0.15f;
static const float temp_scale = 1.0f/333.87f;
static const float temp_offset = 14.0f;

static icm20948_device_t icm_dev;

esp_err_t icm_init(void)
{
    icm20948_device_t icm_dev;
    icm0948_config_i2c_t icm_config = {
        .bus_handle = bus_handle,
        .dev_handle = NULL,
        .i2c_addr = ICM20948_I2C_ADDRESS,
        .i2c_clock_speed = I2C_SPEED,
    };

    bool icm_initialized = true;

    /* setup icm20948 device */
    icm20948_init_i2c(&icm_dev, &icm_config);
    
    /* check ID */
    while (icm20948_check_id(&icm_dev) != ICM_20948_STAT_OK)
    {
        ESP_LOGE(TAG_ICM, "check id failed");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG_ICM, "check id passed");
    
    icm20948_status_e stat;
    
    // Here we are doing a SW reset to make sure the device starts in a known state
    stat = icm20948_sw_reset(&icm_dev);
    if (stat != ESP_OK) {
        ESP_LOGE(TAG_ICM, "reset failed");
        icm_initialized = false;
    }
    vTaskDelay(pdMS_TO_TICKS(250));
    
    // Now wake the sensor up
    stat = icm20948_sleep(&icm_dev, false);
    if (stat != ESP_OK) {
        ESP_LOGE(TAG_ICM, "wake up failed");
        icm_initialized = false;
    }
    stat = icm20948_low_power(&icm_dev, false);
    if (stat != ESP_OK) {
        ESP_LOGE(TAG_ICM, "normal power failed");
        icm_initialized = false;
    }
    
    icm20948_internal_sensor_id_bm sensors = (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);
    
    // Set Gyro and Accelerometer to a particular sample mode
    // options: SAMPLE_MODE_CONTINUOUS. SAMPLE_MODE_CYCLED
    stat = icm20948_set_sample_mode(&icm_dev, sensors, SAMPLE_MODE_CONTINUOUS); 
    if (stat != ESP_OK) {
        ESP_LOGE(TAG_ICM, "set sample mode failed");
        icm_initialized = false;
    }
    
    // Set up sensors sample rate
    icm20948_smplrt_t smplrt;
    smplrt.a = 10;
    smplrt.g = 10;
    stat = icm20948_set_sample_rate(&icm_dev, sensors, smplrt);
    if (stat != ESP_OK) {
        ESP_LOGE(TAG_ICM, "set sample rate failed");
        icm_initialized = false;
    }
    
    // Set full scale ranges for both acc and gyr
    icm20948_fss_t myfss;
    myfss.a = GPM_16;
    myfss.g = DPS_500;
    stat = icm20948_set_full_scale(&icm_dev, sensors, myfss);
    if (stat != ESP_OK) {
        ESP_LOGE(TAG_ICM, "set scale failed");
        icm_initialized = false;
    }
    
    // Set up DLPF configuration
    icm20948_dlpcfg_t myDLPcfg;
    myDLPcfg.a = ACC_D111BW4_N136BW;
    myDLPcfg.g = GYR_D119BW5_B154BW3;
    stat = icm20948_set_dlpf_cfg(&icm_dev, sensors, myDLPcfg);
    if (stat != ESP_OK)
        ESP_LOGE(TAG_ICM, "set DLPF failed");
    
    // Choose whether or not to use DLPF
    stat = icm20948_enable_dlpf(&icm_dev, ICM_20948_INTERNAL_ACC, true);
    if (stat != ESP_OK)
        ESP_LOGE(TAG_ICM, "accel DLPF failed");
    stat = icm20948_enable_dlpf(&icm_dev, ICM_20948_INTERNAL_GYR, true);
    if (stat != ESP_OK)
        ESP_LOGE(TAG_ICM, "gyro DLPF failed");
    
    stat = icm20948_init_magnetometer(&icm_dev);
    if (stat != ESP_OK)
        ESP_LOGE(TAG_ICM, "mag init failed");

    if (icm_initialized)
        return ESP_OK;
    return ESP_FAIL;
}

// void print_agmt(icm20948_agmt_t agmt)
// {
//   	ESP_LOGI("TAG_ICM", "Acc: [ %.5f, %.5f, %.5f ] Gyr: [ %.5f, %.5f, %.5f ] Mag: [ %d, %d, %d ] Tmp: [ %.5f ]", 
// 		agmt.acc.axes.x*acc_scale, agmt.acc.axes.y*acc_scale, agmt.acc.axes.z*acc_scale,
// 		agmt.gyr.axes.x*gyro_scale - gyro_offset_x, agmt.gyr.axes.y*gyro_scale - gyro_offset_y, agmt.gyr.axes.z*gyro_scale - gyro_offset_z,
// 		agmt.mag.axes.x, agmt.mag.axes.y, agmt.mag.axes.z,
// 		agmt.tmp.val*temp_scale + temp_offset
// 	);
// }

void fusion_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/FUSION_SAMPLE_RATE);

    data_t *data = (data_t *)pvParameters;
    icm20948_device_t *icm = &icm_dev;
    
	// Define calibration (replace with actual calibration data if available)
	static const FusionMatrix gyroscopeMisalignment = {{{(1.0f), (0.0f), (0.0f)}, {(0.0f), (1.0f), (0.0f)}, {(0.0f), (0.0f), (1.0f)}}};
	static const FusionVector gyroscopeSensitivity = {{1.0f, 1.0f, 1.0f}};
	static const FusionVector gyroscopeOffset = {{-1.65f, 1.3f, 0.25f}};
	static const FusionMatrix accelerometerMisalignment = {{{1.000599f, 0.000295f, 0.001966f}, 
                                                            {0.000295f, 0.998947f, 0.000291f}, 
                                                            {0.001966f, 0.000291f, 0.984282f}}};
	static const FusionVector accelerometerSensitivity = {{1.0f, 1.0f, 1.0f}};
	static const FusionVector accelerometerOffset = {{-0.002522f, -0.020485f, -0.019608f}};
	static const FusionMatrix softIronMatrix = {{{1.0f, 0.0f, 0.0f},
                                                 {0.0f, 1.0f, 0.0f}, 
                                                 {0.0f, 0.0f, 1.0f}}};
    static const FusionVector hardIronOffset = {{0.0f, 0.0f, 0.0f}};
    
	// Initialise algorithms
	FusionOffset offset;
	FusionAhrs ahrs;

	FusionOffsetInitialise(&offset, FUSION_SAMPLE_RATE);
	FusionAhrsInitialise(&ahrs);

	// Set AHRS algorithm settings
	const FusionAhrsSettings settings = {
        .convention = FusionConventionNed,
        .gain = 0.5f,
        .gyroscopeRange = 500.0f, /* replace this with actual gyroscope range in degrees/s */
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .recoveryTriggerPeriod = 5 * FUSION_SAMPLE_RATE, /* 5 seconds */
	};
	FusionAhrsSetSettings(&ahrs, &settings);
    while(true)
	{
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
        xSemaphoreTake(xI2CMutex, portMAX_DELAY);
        icm20948_agmt_t agmt;
		if (icm20948_get_agmt(icm, &agmt) == ICM_20948_STAT_OK) {
            xSemaphoreGive(xI2CMutex);
            if(initial_temp == 0)
                initial_temp = (agmt.tmp.val*temp_scale + temp_offset) + 273;

			// Acquire latest sensor data
            const clock_t timestamp = esp_timer_get_time();
            FusionVector gyroscope = {{agmt.gyr.axes.x*gyro_scale, agmt.gyr.axes.y*gyro_scale, agmt.gyr.axes.z*gyro_scale}}; // in degrees/s
            FusionVector accelerometer = {{agmt.acc.axes.x*acc_scale, agmt.acc.axes.y*acc_scale, agmt.acc.axes.z*acc_scale}}; // in g
            FusionVector magnetometer = {{agmt.mag.axes.x*mag_scale, agmt.mag.axes.y*mag_scale, agmt.mag.axes.z*mag_scale}}; // in arbitrary units

            // Apply calibration
            gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
            accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
            magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

            // Update gyroscope offset correction algorithm
            gyroscope = FusionOffsetUpdate(&offset, gyroscope);

            // Calculate delta time (in seconds) to account for gyroscope sample clock error
            static clock_t previousTimestamp;
            const float deltaTime = (float) (timestamp - previousTimestamp) / (float) CLOCKS_PER_SEC;
            previousTimestamp = timestamp;

            // Update gyroscope AHRS algorithm
            FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

            const FusionVector a = FusionAhrsGetLinearAcceleration(&ahrs);
            float accel = sqrt(powf(a.axis.x, 2) + powf(a.axis.y, 2) + powf(a.axis.z, 2));
            
            const FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs);
            
            xSemaphoreTake(xDataMutex, portMAX_DELAY);
            data->accel = (uint8_t)floor(accel*100);
            data->orientation_q1 = q.element.z;
            data->orientation_q2 = q.element.w;
            data->orientation_q3 = q.element.x;
            data->orientation_q4 = q.element.y;

            // Update raw sensor data
            data->accel_x = agmt.acc.axes.x;
            data->accel_x = agmt.acc.axes.x;
            data->accel_x = agmt.acc.axes.x;
            
            data->gyro_x = agmt.gyr.axes.x;
            data->gyro_x = agmt.gyr.axes.x;
            data->gyro_x = agmt.gyr.axes.x;
            
            data->mag_x = agmt.mag.axes.x;
            data->mag_x = agmt.mag.axes.x;
            data->mag_x = agmt.mag.axes.x;

            data->temperature = agmt.tmp.val;
            xSemaphoreGive(xDataMutex);

            const FusionVector g = FusionAhrsGetEarthAcceleration(&ahrs);
            float acc = acc_mahalanobis(-g.axis.z);
            if (STATUS & ARMED)
                altitude_predict(acc);
            // print_agmt(agmt);
		} else {
			ESP_LOGE(TAG_FUSION, "get agmt failed");
		}
    }
    vTaskDelete(NULL);
}