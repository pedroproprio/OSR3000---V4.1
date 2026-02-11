#include "header.h"

static const char *TAG_ACQ = "ACQUIRE";
static const char *TAG_DEPLOY = "DEPLOY";

#define HIGH 1
#define LOW 0

static void adc_init(adc_oneshot_unit_handle_t *adc_unit_handle, adc_cali_handle_t *adc_cali_handle) {
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = VIN_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    adc_oneshot_new_unit(&unit_config, adc_unit_handle);

    // Configure the ADC channel
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,             
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_unit_handle, VIN_ADC_CHANNEL, &channel_config));

    // Configure calibration (raw ADC value in mV)
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = VIN_ADC_UNIT,
        .chan = VIN_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, adc_cali_handle));
}

void task_adc(void *pvParameters) {
    adc_oneshot_unit_handle_t adc_unit_handle;
    adc_cali_handle_t adc_cali_handle;
    adc_init(&adc_unit_handle, &adc_cali_handle); // Initialize ADC

    while (true)
    {
        int raw, voltage_mv;
        esp_err_t err = (adc_oneshot_read(adc_unit_handle, VIN_ADC_CHANNEL, &raw));
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_ACQ, "ADC read failed: %s", esp_err_to_name(err));
            continue;
        }

        err = (adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv));
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_ACQ, "ADC calibration failed: %s", esp_err_to_name(err));
            continue;
        }

        // Calculate actual battery voltage based on voltage divider ratio
        portENTER_CRITICAL(&xADCMutex);
        battery_voltage_g = (uint8_t)round((voltage_mv * V_DIV_RATIO)); //  Apply divider ratio, save as V * 10
        portEXIT_CRITICAL(&xADCMutex);
        
        xTaskNotifyGive(xTaskAcquire);

        vTaskDelay(pdMS_TO_TICKS(10000)); // Read every 10 seconds
    }
}

void status_check(data_t *data)
{
    // Check if altitude is higher than FLYING_THRESHOLD
    if (!(data->status & FLYING) && data->kf.initial_altitude != 0)
    {
        if (fabs(data->kf.altitude) > data->kf.initial_altitude + FLYING_THRESHOLD)
            data->status |= FLYING;
    }

    // Check if accel is lower than CUTOFF_THRESHOLD
    if ((data->status & FLYING) && !(data->status & CUTOFF))
    {
        if (data->icm.accel < CUTOFF_THRESHOLD)
            data->status |= CUTOFF;
    }

    // Check if landed by comparing altitude to altitude 5 seconds ago
    if ((data->status & FLYING) && !(data->status & LANDED))
    {
        static float aux_altitude = 0;
        static int64_t aux_time = 0;
        if (esp_timer_get_time() - aux_time > 2000000LL) // If 2 seconds have passed since last check
        {
            if (aux_time == 0) // If first time checking, set aux_time and aux_altitude
            {
                aux_time = esp_timer_get_time();
                aux_altitude = data->kf.altitude;
            }
            else if (fabs(data->kf.altitude - aux_altitude) < LANDED_THRESHOLD) // If altitude has not changed more than LANDED_THRESHOLD, consider landed
            {
                data->status |= LANDED;
            }
            else // If altitude has changed more than LANDED_THRESHOLD, update aux_time and aux_altitude
            {
                aux_time = esp_timer_get_time();
                aux_altitude = data->kf.altitude;
            }
        }
    }
}

static void pack_save_data(const data_t *data, save_t *save_data)
{
    save_data->time = data->time;
    save_data->pressure = data->bmp.pressure;
    save_data->latitude = data->gps.latitude;
    save_data->longitude = data->gps.longitude;
    save_data->gps_altitude = data->gps.altitude;
    save_data->gps_vel_vertical = data->gps.vel_vertical;
    save_data->temperature = data->icm.temperature;
    save_data->accel_x = data->icm.accel_x;
    save_data->accel_y = data->icm.accel_y;
    save_data->accel_z = data->icm.accel_z;
    save_data->gyro_x = data->icm.gyro_x;
    save_data->gyro_y = data->icm.gyro_y;
    save_data->gyro_z = data->icm.gyro_z;
    save_data->mag_x = data->icm.mag_x;
    save_data->mag_y = data->icm.mag_y;
    save_data->mag_z = data->icm.mag_z;
    save_data->status = data->status;
    save_data->voltage = data->voltage;
}

static void pack_send_data(const data_t *data, send_t *send_data)
{
    send_data->time = data->time;
    send_data->latitude = data->gps.latitude;
    send_data->longitude = data->gps.longitude;
    send_data->q1 = data->icm.q1;
    send_data->q2 = data->icm.q2;
    send_data->q3 = data->icm.q3;
    send_data->q4 = data->icm.q4;
    send_data->kf_altitude = data->kf.altitude;
    send_data->kf_vel_vertical = data->kf.vel_vertical;
    send_data->kf_apogee = data->kf.apogee;
    send_data->accel = data->icm.accel;
    send_data->status = data->status;
    send_data->voltage = data->voltage;
}

// send_queues sends data to queues
void send_queues(const data_t *data)
{    
    if (!(data->status & LANDED)) // If not landed
    {
        save_t save_data;
        pack_save_data(data, &save_data);
        
        xQueueSend(xSDQueue, &save_data, 0);  // Send to SD card queue
        if (!(data->status & LFS_FULL)) // If LittleFS is not full, send to LittleFS queue
            xQueueSend(xLittleFSQueue, &save_data, 0);
    }
    send_t send_data;
    pack_send_data(data, &send_data);
    xQueueOverwrite(xLoraQueue, &send_data); // Send to LoRa queue
}

// task_deploy deploys parachutes
void task_deploy(void *pvParameters)
{
    data_t data;

    while (true)
    {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for being armed
    ESP_LOGW(TAG_DEPLOY, "System armed. Deploy active.");

        while (true)
        {
            portENTER_CRITICAL(&xDATAMutex);
            data = data_g;
            portEXIT_CRITICAL(&xDATAMutex);
            
            // Disarm condition
            if (!(data.status & ARMED))
            {
                ESP_LOGW(TAG_DEPLOY, "System disarmed.");
                
                gpio_set_level(LED_GPIO, HIGH);
                gpio_set_level(BUZZER_GPIO, HIGH);
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level(LED_GPIO, LOW);
                gpio_set_level(BUZZER_GPIO, LOW);
                vTaskDelay(pdMS_TO_TICKS(1000));
                for (int i = 0; i < 3; ++i)
                {
                    gpio_set_level(LED_GPIO, HIGH);
                    gpio_set_level(BUZZER_GPIO, HIGH);
                    vTaskDelay(pdMS_TO_TICKS(150));
                    gpio_set_level(LED_GPIO, LOW);
                    gpio_set_level(BUZZER_GPIO, LOW);
                    vTaskDelay(pdMS_TO_TICKS(950));
                }
                
                break; // Continues if the system is not armed
            }
            
            // Drogue not deployed and altitude is below apogee - DROGUE_THRESHOLD
            bool drogue_caindo = !(data.status & DROGUE_DEPLOYED) &&
                (data.kf.altitude < data.kf.apogee - DROGUE_THRESHOLD);
            // Main not deployed, drogue deployed and altitude is below initial_altitude + MAIN_ALTITUDE
            bool main_caindo = (data.status & DROGUE_DEPLOYED) && !(data.status & MAIN_DEPLOYED) &&
                (data.kf.altitude < data.kf.initial_altitude + MAIN_ALTITUDE);

            if (drogue_caindo)
            {            
                portENTER_CRITICAL(&xDATAMutex);
                data_g.status |= DROGUE_DEPLOYED;
                portEXIT_CRITICAL(&xDATAMutex);

                gpio_set_level(DROGUE_GPIO, HIGH);
                ESP_LOGW(TAG_DEPLOY, "Drogue deployed");
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(DROGUE_GPIO, LOW);
            }

            else if (main_caindo)
            {
                portENTER_CRITICAL(&xDATAMutex);
                data_g.status |= MAIN_DEPLOYED;
                portEXIT_CRITICAL(&xDATAMutex);

                gpio_set_level(MAIN_GPIO, HIGH);
                ESP_LOGW(TAG_DEPLOY, "Main deployed");
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(MAIN_GPIO, LOW);
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

void task_acquire(void *pvParameters)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for notification from reading tasks
        
        data_t data;
        bmp390_sample_t bmp;
        icm20948_sample_t icm;
        gps_sample_t gps;

        portENTER_CRITICAL(&xBMPMutex);
        bmp = bmp_sample_g;
        portEXIT_CRITICAL(&xBMPMutex);

        portENTER_CRITICAL(&xICMMutex);
        icm = icm_sample_g;
        portEXIT_CRITICAL(&xICMMutex);

        portENTER_CRITICAL(&xGPSMutex);
        gps = gps_sample_g;
        portEXIT_CRITICAL(&xGPSMutex);

        portENTER_CRITICAL(&xADCMutex);
        data.voltage = battery_voltage_g;
        portEXIT_CRITICAL(&xADCMutex);

        /* get kalman states */

        data.bmp = bmp;
        data.icm = icm;
        data.gps = gps;
        // data.kf = Replace with actual Kalman filter state vector

        portENTER_CRITICAL(&xDATAMutex);
        data.status = data_g.status;
        portEXIT_CRITICAL(&xDATAMutex);

        status_check(&data); // status check and update
        
        // get time
        if (gps.utc_time > 1) // Register GPS time only once, if available (not 0 and not 1) 
        {
            data.time = (uint32_t)(gps.utc_time);
            portENTER_CRITICAL(&xGPSMutex);
            gps_sample_g.utc_time = 1; // Set to 1 to indicate GPS time has been registered
            portEXIT_CRITICAL(&xGPSMutex);
        }
        else
            data.time = (uint32_t)(esp_timer_get_time() / 1000UL); // ms since boot
        
        send_queues(&data); // sd, littlefs and lora queues
        
        portENTER_CRITICAL(&xDATAMutex);
        data_g = data; // Update global data with latest data
        portEXIT_CRITICAL(&xDATAMutex);

        xTaskNotifyGive(xTaskDeploy); // Notify deploy task to check for deployment conditions
    }
}

void task_log(void *pvParameters)
{
    data_t data;
    while (true)
    {
        portENTER_CRITICAL(&xDATAMutex);
        data = data_g;
        portEXIT_CRITICAL(&xDATAMutex);

        ESP_LOGI(TAG_ACQ, 
            "\n\tTime (ms):\t\t%lu\r\n"
            "\tStatus:\t\t\t%d\r\n"
            "\tVoltage (V):\t\t%.1f\r\n"
            "\tBMP pressure (Pa):\t%.2f\r\n"
            "\tBMP altitude (m):\t%.2f\r\n"
            "\tBMP initial alt (m):\t%.2f\r\n"
            "\tTemperature (°C):\t%.2f\r\n"
            "\tAccel (g):\t\tax: %.2f, ay: %.2f, az: %.2f\r\n"
            "\tGyro (°/s):\t\tgx: %.2f, gy: %.2f, gz: %.2f\r\n"
            "\tMag (µT):\t\tmx: %.2f, my: %.2f, mz: %.2f\r\n"
            "\t|Accel| (g):\t\t%.1f\r\n"
            "\tGPS coord (°):\t\tLat: %.5f, Lon: %.5f\r\n"
            "\tGPS altitude (m):\t%.2f\r\n"
            "\tGPS velD (m/s):\t\t%.2f\r\n"
            "\tGPS initial alt (m):\t%.2f\r\n"
            "\tOrientation\t\tq1: %.5f, q2: %.5f, q3: %.5f, q4: %.5f\r\n"
            "\tKalman alt (m):\t\t%.2f\r\n"
            "\tKalman velD (m/s):\t%.2f\r\n"
            "\tKalman apogee (m):\t%.2f\r\n"
            "\tKalman initial alt (m):\t%.2f\r\n"
            "-------------------------------------------------------------------------------------",
            data.time,
            data.status,
            data.voltage*0.1f,
            data.bmp.pressure,
            data.bmp.altitude,
            data.bmp.initial_altitude,
            data.icm.temperature*TEMP_SCALE + TEMP_OFFSET,
            data.icm.accel_x*ACC_SCALE, data.icm.accel_y*ACC_SCALE, data.icm.accel_z*ACC_SCALE,
            data.icm.gyro_x*GYRO_SCALE, data.icm.gyro_y*GYRO_SCALE, data.icm.gyro_z*GYRO_SCALE,
            data.icm.mag_x*MAG_SCALE, data.icm.mag_y*MAG_SCALE, data.icm.mag_z*MAG_SCALE,
            data.icm.accel*0.1f,
            data.gps.latitude, data.gps.longitude,
            data.gps.altitude,
            data.gps.vel_vertical,
            data.gps.initial_altitude,
            data.icm.q1, data.icm.q2, data.icm.q3, data.icm.q4,
            data.kf.altitude,
            data.kf.vel_vertical,
            data.kf.apogee,
            data.kf.initial_altitude
        );
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}