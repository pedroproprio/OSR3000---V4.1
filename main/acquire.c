#include "header.h"

static const char *TAG_ACQ = "ACQUIRE";
static const char *TAG_DEPLOY = "DEPLOY";

#define sACC_THRESHOLD 0.5f // Threshold for GPS sAcc above which xR is scaled up in eskf_update_gps

#define HIGH 1
#define LOW 0

// Contains logic for state transitions and deployments
void status_check(data_t *data)
{
    // ===================== SAFE MODE =====================
    if ((data->status & ARMED) && !(data->status & BOOST) && gpio_get_level(RBF_GPIO) == LOW)
    {
        portENTER_CRITICAL(&xDATAMutex);
        data_g.status &= ~(ARMED);
        portEXIT_CRITICAL(&xDATAMutex);
        
        ESP_LOGW(TAG_ACQ, "Disarming system");
        gpio_set_level(LED_GPIO, HIGH);
        gpio_set_level(BUZZER_GPIO, HIGH);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED_GPIO, LOW);
        gpio_set_level(BUZZER_GPIO, LOW);
    }
    
    // ===================== ARMED =====================
    else if (!(data->status & ARMED) && gpio_get_level(RBF_GPIO) == HIGH)
    {
        portENTER_CRITICAL(&xDATAMutex);
        data_g.status |= ARMED;
        portEXIT_CRITICAL(&xDATAMutex);

        ESP_LOGW(TAG_ACQ, "System ARMED");
        for (uint8_t i = 0; i < 3; i++)
        {
            gpio_set_level(LED_GPIO, HIGH);
            gpio_set_level(BUZZER_GPIO, HIGH);
            vTaskDelay(pdMS_TO_TICKS(150));
            gpio_set_level(LED_GPIO, LOW);
            gpio_set_level(BUZZER_GPIO, LOW);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
    }

    // ===================== BOOST =====================
    if (!(data->status & BOOST))
    {
        static uint32_t t_boost = 0;
        // Vertical acceleration greater then threshold and positive vertical velocity
        if (data->icm.az > BOOST_THRESHOLD_A && data->kf.x.vz > 0.0f)
        {
            if (t_boost == 0)
                t_boost = data->time;
            // If condition holds, set BOOST flag
            if ((data->time - t_boost) > THRESHOLD_MS)
                data->status |= BOOST;
        }
        else
            t_boost = 0;
        return;
    }

    // ===================== COAST =====================
    if ((data->status & BOOST) && !(data->status & COAST))
    {
        static uint32_t t_coast = 0;
        // Vertical acceleration less than threshold
        if (data->icm.az < COAST_THRESHOLD_A)
        {
            if (t_coast == 0)
                t_coast = data->time;
            // If condition holds, set COAST flag
            if ((data->time - t_coast) > THRESHOLD_MS)
                data->status |= COAST;
        }
        else
            t_coast = 0;
        return;
    }

    // ===================== DROGUE DEPLOY =====================
    if ((data->status & ARMED) && (data->status & COAST) && !(data->status & DROGUE_DEPLOYED))
    {
        static uint32_t t_apogee = 0;
        // Absolute vertical velocity is less than threshold (near apogee)
        if (fabsf(data->kf.x.vz) < DROGUE_THRESHOLD_V)
        {
            if (t_apogee == 0)
                t_apogee = data->time;
            // If condition holds and altitude is less than apogee, set deploy drogue
            if ((data->time - t_apogee) > THRESHOLD_MS && data->kf.x.h < data->kf.x.apogee)
            {
                data->status |= DROGUE_DEPLOYED;
                gpio_set_level(DROGUE_GPIO, HIGH);
                ESP_LOGW(TAG_DEPLOY, "Drogue deployed");
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(DROGUE_GPIO, LOW);
            }
        }
        else
            t_apogee = 0;
        return;
    }

    // ===================== MAIN DEPLOY =====================
    if ((data->status & DROGUE_DEPLOYED) && !(data->status & MAIN_DEPLOYED))
    {
        // Deploy main if altitude is less than MAIN_ALTITUDE and vertical velocity is negative (descending)
        if (data->kf.x.h < MAIN_ALTITUDE && data->kf.x.vz < 0)
        {
            data->status |= MAIN_DEPLOYED;
            gpio_set_level(MAIN_GPIO, HIGH);
            ESP_LOGW(TAG_DEPLOY, "Main deployed");
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(MAIN_GPIO, LOW);
        }
        return;
    }

    // ===================== LANDING =====================
    if ((data->status & MAIN_DEPLOYED) && !(data->status & LANDING))
    {
        static uint32_t t_landing = 0;
        // Time to impact is less than PREPARE_FOR_LANDING_S seconds (h/vz) and descending
        if (data->kf.x.h / -data->kf.x.vz < PREPARE_FOR_LANDING_S)
        {
            if (t_landing == 0)
                t_landing = data->time;
            // If condition holds, set LANDING flag
            if ((data->time - t_landing) > THRESHOLD_MS)
                data->status |= LANDING;
            else
                t_landing = 0;
        }
        return;
    }

    // ===================== LANDED =====================
    if (data->status & LANDING && !(data->status & LANDED))
    {
        static uint32_t t_landed = 0;
        // Absolute vertical velocity and acceleration are near zero
        if (fabsf(data->kf.x.vz) < 0.5f && data->icm.accel < 1.0f)
        {
            if (t_landed == 0)
                t_landed = data->time;
            // If condition holds, set LANDED flag
            if ((data->time - t_landed) > 10*THRESHOLD_MS)
                data->status |= LANDED;
            else
                t_landed = 0;
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
    save_data->sAcc = data->gps.sAcc;
    save_data->fix = data->gps.fix;
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
    send_data->fix = data->gps.fix;
    send_data->q1 = data->icm.q1;
    send_data->q2 = data->icm.q2;
    send_data->q3 = data->icm.q3;
    send_data->q4 = data->icm.q4;
    send_data->kf_altitude = data->kf.x.h;
    send_data->kf_vel_vertical = data->kf.x.vz;
    send_data->kf_apogee = data->kf.x.apogee;
    send_data->accel = data->icm.accel;
    send_data->status = data->status;
    send_data->voltage = data->voltage;
}

// Sends data to SD card, LittleFS and LoRa queues
void send_queues(const data_t *data)
{
    if (!(data->status & BOOST)) // If in idle state, do not save data to save resources
    {
        save_t save_data;
        pack_save_data(data, &save_data);
        xQueueSend(xB4LaunchQueue, &save_data, 0); // This queue will be only saved at landing
    }
    else if (!(data->status & LANDING)) // If not approaching ground
    {
        save_t save_data;
        pack_save_data(data, &save_data);
        
        xQueueSend(xSDQueue, &save_data, 0); // Send to SD card queue
        if (!atomic_load_explicit(&lfs_full, memory_order_relaxed)) // If LittleFS is not full, send to LittleFS queue
            xQueueSend(xLittleFSQueue, &save_data, 0);
    }
    send_t send_data;
    pack_send_data(data, &send_data);
    xQueueOverwrite(xLoraQueue, &send_data); // Send to LoRa queue (length 1)
}

void task_acquire(void *pvParameters)
{
    uint32_t notifiedValue;
    while (true)
    {
        // Wait for notification from reading tasks
        xTaskNotifyWait(
            0,                // don't clear any bits on entry
            UINT32_MAX,       // clear all bits on exit
            &notifiedValue, portMAX_DELAY);
        
        data_t data;

        portENTER_CRITICAL(&xDATAMutex);
        data = data_g;
        portEXIT_CRITICAL(&xDATAMutex);

        portENTER_CRITICAL(&xBMPMutex);
        data.bmp = bmp_sample_g;
        portEXIT_CRITICAL(&xBMPMutex);

        portENTER_CRITICAL(&xICMMutex);
        data.icm = icm_sample_g;
        portEXIT_CRITICAL(&xICMMutex);

        portENTER_CRITICAL(&xGPSMutex);
        data.gps = gps_sample_g;
        portEXIT_CRITICAL(&xGPSMutex);

        portENTER_CRITICAL(&xADCMutex);
        data.voltage = battery_voltage_g;
        portEXIT_CRITICAL(&xADCMutex);
        
        // Update time
        if (data.gps.utc_time > 1) // Register GPS time only once, if available (not 0 and not 1) 
        {
            data.time = (uint32_t)(data.gps.utc_time);
            portENTER_CRITICAL(&xGPSMutex);
            gps_sample_g.utc_time = 1; // Set to 1 to indicate GPS time has been registered
            portEXIT_CRITICAL(&xGPSMutex);
        }
        else
            data.time = (uint32_t)(esp_timer_get_time() / 1000UL); // ms since boot

        status_check(&data); // Status check and update

        switch (notifiedValue)
        { 
            case ICM_BIT:
                // Trust accelerometer less due to vibration if in boost phase
                float xQ = data.status & BOOST ? 25.0f : 1.0f;
                eskf_predict(&data.kf, data.icm.az, xQ);
                break;
            case BMP_BIT:
                if (!eskf_update_bar(&data.kf, data.bmp.altitude, 1.0f))
                    ESP_LOGW(TAG_ACQ, "kf_update_bar skipped");
                break;
            case GPS_BIT:
                float sAcc = (float)data.gps.sAcc * 0.1f; // Convert back to m/s
                // Trust GPS less if speed accuracy is greater than threshold (2+sAcc-sACC_THRESHOLD)²
                float xR = (sAcc > sACC_THRESHOLD) ? (2.0f + sAcc - sACC_THRESHOLD) * (2.0f + sAcc - sACC_THRESHOLD): 1.0f;
                if (!eskf_update_gps(&data.kf, data.gps.altitude, data.gps.vel_vertical, xR))
                    ESP_LOGW(TAG_ACQ, "kf_update_gps skipped");
                break;
            case ADC_BIT:
                break;
            default:
                ESP_LOGW(TAG_ACQ, "Unknown notification received: %s", uint32_to_binary(notifiedValue));
        }
        
        send_queues(&data); // SD, littleFS and lora queues
        
        portENTER_CRITICAL(&xDATAMutex);
        data_g = data; // Update global data with latest data
        portEXIT_CRITICAL(&xDATAMutex);
    }
}