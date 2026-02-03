#include "header.h"

static const char *TAG_ACQ = "ACQUIRE";

void adc_init(adc_oneshot_unit_handle_t *adc_unit_handle, adc_cali_handle_t *adc_cali_handle) {
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    adc_oneshot_new_unit(&unit_config, adc_unit_handle);

    // Configure the ADC channel
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,             
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_unit_handle, ADC_CHANNEL_4, &channel_config));

    // Configure calibration (raw ADC value in mV)
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_4,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, adc_cali_handle));
}

float read_battery_voltage(adc_oneshot_unit_handle_t adc_unit_handle, adc_cali_handle_t adc_cali_handle) {
    int raw;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_unit_handle, ADC_CHANNEL_4, &raw));

    int voltage_mv;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv));

    // Calculate actual battery voltage based on voltage divider ratio
    float battery_voltage = (voltage_mv / 1000.0f) * ((R1 + R2) / R2); // Convert mV to V and apply divider ratio
    // (R1 + R2) / R2 = 1.5 for R1=10k and R2=20k
    return battery_voltage;
}

void status_checks(data_t *data)
{
    // Check if altitude is higher than KNOWN_ALTITUDE + FLYING_THRESHOLD
    if (!(data->status & FLYING))
    {
        if (fabs(data->bmp_altitude) > KNOWN_ALTITUDE + FLYING_THRESHOLD)
        {
            xSemaphoreTake(xStatusMutex, portMAX_DELAY);
            STATUS |= FLYING;
            xSemaphoreGive(xStatusMutex);
        }
    }

    // Check if accel is lower than CUTOFF_THRESHOLD
    if ((data->status & FLYING) && !(data->status & CUTOFF))
    {
        if (data->accel*0.1 < CUTOFF_THRESHOLD)
        {
            xSemaphoreTake(xStatusMutex, portMAX_DELAY);
            STATUS |= CUTOFF;
            xSemaphoreGive(xStatusMutex);
        }
    }

    // Check if landed by comparing altitude to altitude 5 seconds ago
    if ((data->status & FLYING) && !(data->status & LANDED))
    {
        static float aux_altitude = 0;
        static int64_t aux_time = 0;
        if (esp_timer_get_time() - aux_time > 5000000) // If 5 seconds have passed since last check
        {
            if (aux_time == 0) // If first time checking, set aux_time and aux_altitude
            {
                aux_time = esp_timer_get_time();
                aux_altitude = data->bmp_altitude;
            }
            else if (fabs(data->bmp_altitude - aux_altitude) < LANDED_THRESHOLD) // If altitude has not changed more than LANDED_THRESHOLD, consider landed
            {
                xSemaphoreTake(xStatusMutex, portMAX_DELAY);
                STATUS |= LANDED;
                xSemaphoreGive(xStatusMutex);
            }
            else // If altitude has changed more than LANDED_THRESHOLD, update aux_time and aux_altitude
            {
                aux_time = esp_timer_get_time();
                aux_altitude = data->bmp_altitude;
            }
        }
    }
}

// send_queues sends data to queues
void send_queues(data_t *data)
{
    if (!(data->status & LANDED)) // If not landed, send to queues
    {
        if ((data->status & ARMED)) // If armed, send to task_deploy
            xQueueSend(xAltQueue, &data->bmp_altitude, 0);
        data_t save_data;
        save_struct(data, &save_data);
        xQueueSend(xSDQueue, &save_data, 0);  // Send to SD card queue
        if (!(data->status & LFS_FULL)) // If LittleFS is not full, send to LittleFS queue
            xQueueSend(xLittleFSQueue, &save_data, 0);
    }

    static int n = 0;
    if (n++ % 10 == 0) // This affects the frequency of the LoRa messages
    {
        data_t send_data;
        data->kf_altitude = altitude_get_alt();
        data->kf_vel_vertical = altitude_get_vel();
        send_struct(data, &send_data);
        xQueueSend(xLoraQueue, &send_data, 0); // Send to LoRa queue
    }

    ESP_LOGI(TAG_ACQ, "Data sent to queues");
}

void task_acquire(void *pvParameters)
{
    xI2CMutex = xSemaphoreCreateMutex();
    data_t data = {0};

    // ADC Initialization
    adc_oneshot_unit_handle_t adc_unit_handle;
    adc_cali_handle_t adc_cali_handle;
    adc_init(&adc_unit_handle, &adc_cali_handle);

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, GPS_BUFF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, GPS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uint8_t buffer[GPS_BUFF_SIZE];

    ESP_ERROR_CHECK(bmp_init());
    ESP_ERROR_CHECK(icm_init());

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
    icm20948_agmt_t agmt;
    TickType_t xLastWakeTime = 0;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/FUSION_SAMPLE_RATE);

    while (true)
    {
        // Time and status update
        if (utc_time != 0) {
            data.time = (uint32_t)(utc_time);
            utc_time = 0;
        }
        else data.time = (uint32_t)(esp_timer_get_time() * 0.001);
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        data.status = STATUS;
        xSemaphoreGive(xStatusMutex);

        // Battery voltage
        data.voltage = read_battery_voltage(adc_unit_handle, adc_cali_handle);

        gps_task(&data, buffer);
        bmp_task(&data);
        xLastWakeTime = xTaskGetTickCount();
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
        fusion_task(&data, &offset, &ahrs, &agmt);

        status_checks(&data);

        // Print data
        ESP_LOGI(TAG_ACQ, "\tTime: %lu, \tStatus: %d V: %u\r\n"
                          "\tBMP\t\tP: %.2f, T: %u, A: %.2f\r\n"
                          "\tAccel\t\tX: %d, Y: %d, Z: %d\r\n"
                          "\tGyro\t\tX: %d, Y: %d, Z: %d\r\n"
                          "\tMag\t\tX: %d, Y: %d, Z: %d\r\n"
                          "\tGPS\t\tLat: %.5f, Lon: %.5f, Alt: %.2f, DVel: %.2f\r\n"
                          "\tTotal Accel\t\tG: %d\r\n"
                          "\tOrientation\t\tQ1: %.5f, Q2: %.5f, Q3: %.5f, Q4: %.5f\r\n"
                          "\tKalman\t\talt: %.2f, DVel: %.2f\n"
                          "----------------------------------------",
                data.time, data.status, data.voltage,
                data.pressure, data.temperature, data.bmp_altitude,
                data.accel_x, data.accel_y, data.accel_z,
                data.gyro_x, data.gyro_y, data.gyro_z,
                data.mag_x, data.mag_y, data.mag_z,
                data.latitude, data.longitude, data.gps_altitude, data.gps_vel_vertical,
                data.accel,
                data.orientation_q1, data.orientation_q2, data.orientation_q3, data.orientation_q4,
                data.kf_altitude, data.kf_vel_vertical);
        send_queues(&data);

        // REDUCE AFTER OPTIMIZING CODE
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

void send_struct(const data_t *src, data_t *dst){
    if (!src || !dst) return;
    dst->time = src->time;
    dst->status = src->status;
    dst->voltage = src->voltage;
    dst->latitude = src->latitude;
    dst->longitude = src->longitude;
    dst->gps_altitude = src->gps_altitude - gps_initial_alt; // relative to initial state
    dst->gps_vel_vertical = src->gps_vel_vertical;
    dst->bmp_altitude = src->bmp_altitude - bmp_initial_alt; // relative to initial state
    dst->accel = src->accel;
    dst->orientation_q1 = src->orientation_q1;
    dst->orientation_q2 = src->orientation_q2;
    dst->orientation_q3 = src->orientation_q3;
    dst->orientation_q4 = src->orientation_q4;
    dst->kf_altitude = src->kf_altitude;
    dst->kf_vel_vertical = src->kf_vel_vertical;
}

void save_struct(const data_t *src, data_t *dst){
    if (!src || !dst) return;
    dst->time = src->time;
    dst->status = src->status;
    dst->voltage = src->voltage;
    dst->latitude = src->latitude;
    dst->longitude = src->longitude;
    dst->gps_altitude = src->gps_altitude;
    dst->gps_vel_vertical = src->gps_vel_vertical;
    dst->pressure = src->pressure;
    dst->temperature = src->temperature;
    dst->accel_x = src->accel_x;
    dst->accel_y = src->accel_y;
    dst->accel_z = src->accel_z;
    dst->gyro_x = src->gyro_x;
    dst->gyro_y = src->gyro_y;
    dst->gyro_z = src->gyro_z;
    dst->mag_x = src->mag_x;
    dst->mag_y = src->mag_y;
    dst->mag_z = src->mag_z;
}