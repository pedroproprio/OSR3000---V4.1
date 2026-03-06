#include "header.h"

#define HIGH 1
#define LOW 0

static const char *TAG_MAIN = "MAIN";

// task_buzzer_led blinks LED and beeps buzzer to indicate status
void task_buzzer_led(void *pvParameters)
{
    while (true)
    {
        bool landed = false;
        portENTER_CRITICAL(&xDATAMutex);
        if (data_g.status & LANDED)
            landed = true;
        portEXIT_CRITICAL(&xDATAMutex);

        gpio_set_level(LED_GPIO, HIGH);
        if (landed) gpio_set_level(BUZZER_GPIO, HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_GPIO, LOW);
        if (landed) gpio_set_level(BUZZER_GPIO, LOW);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void setup_peripherals(void)
{
    // I2C bus configuration
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    // I2C bus initialization (shared by BMP390 and ICM20948)
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    // GPIO Initialization
    gpio_set_direction(RBF_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RBF_GPIO, GPIO_PULLUP_ONLY);

    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(DROGUE_GPIO);
    gpio_set_direction(DROGUE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DROGUE_GPIO, LOW); // Ensure drogue is not deployed at startup
    gpio_reset_pin(MAIN_GPIO);
    gpio_set_direction(MAIN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MAIN_GPIO, LOW); // Ensure main is not deployed at startup

    // Signal when initializing
    gpio_set_level(LED_GPIO, HIGH);
    gpio_set_level(BUZZER_GPIO, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LED_GPIO, LOW);
    gpio_set_level(BUZZER_GPIO, LOW);
}

static bool check_for_format_mode(void)
{
    if (gpio_get_level(BUTTON_GPIO) == LOW)
    {
        int64_t time = esp_timer_get_time();
        while (gpio_get_level(BUTTON_GPIO) == LOW)
        {
            if (esp_timer_get_time() - time > 5000000LL)
            {
                ESP_LOGW("RESET", "Button pressed for 5 seconds. Formatting...");
                // Signal format mode
                for (uint32_t i = 0; i < 2; ++i)
                {
                    gpio_set_level(LED_GPIO, HIGH);
                    gpio_set_level(BUZZER_GPIO, HIGH);
                    vTaskDelay(pdMS_TO_TICKS(150));
                    gpio_set_level(LED_GPIO, LOW);
                    gpio_set_level(BUZZER_GPIO, LOW);
                    vTaskDelay(pdMS_TO_TICKS(150));
                }
                return true;
            }
            vTaskDelay(10);
        }
    }
    return false;
}

static void manage_nvs_counters(bool format_mode)
{
    // Initialize NVS to store file counters
    esp_err_t err = nvs_flash_init();

    if (err != ESP_OK)
    {
        // Retry nvs_flash_init
        ESP_LOGE("NVS", "%s, erasing NVS partition...", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    // Open NVS
    nvs_handle_t nvs_handle;
    ESP_LOGI("NVS", "Opening Non-Volatile Storage (NVS) handle... ");
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));

    int32_t sd_num = 0;
    int32_t lfs_num = 0;

    nvs_get_i32(nvs_handle, "sd_counter", &sd_num);
    nvs_get_i32(nvs_handle, "lfs_counter", &lfs_num);

    if (format_mode)
    {
        sd_num = 0;
        lfs_num = 0;
    }

    nvs_close(nvs_handle);

    // Save counters to global struct
    file_counter_g.file_numSD = sd_num;
    file_counter_g.file_numLFS = lfs_num;
    file_counter_g.format = format_mode;
}

void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "Starting main application");
    setup_peripherals();
    vTaskDelay(pdMS_TO_TICKS(150)); // Wait for peripherals to stabilize

    // Create Mutexes
    xI2CMutex = xSemaphoreCreateMutex();
    xNVSCounterEvent = xEventGroupCreate();
    xFormatEvent = xEventGroupCreate();

    // Create Queues
    xSDQueue = xQueueCreate(SD_QUEUE_SIZE, sizeof(save_t));
    xLittleFSQueue = xQueueCreate(LITTLEFS_QUEUE_SIZE, sizeof(save_t));
    xLoraQueue = xQueueCreate(LORA_QUEUE_SIZE, sizeof(send_t));
    xB4LaunchQueue = xQueueCreate(B4LAUNCH_QUEUE_SIZE, sizeof(save_t));

    const EventBits_t bits_to_wait = EVT_SD_DONE | EVT_LFS_DONE;

    do {
        bool format_mode = check_for_format_mode();
        manage_nvs_counters(format_mode);
        // If format is true, format SD and LittleFS, then restart
        if (format_mode)
        {
            xTaskCreate(task_sd, "SD", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 5, NULL);
            xTaskCreate(task_littlefs, "LittleFS", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 5, NULL);
            xEventGroupWaitBits(xFormatEvent, bits_to_wait, pdTRUE, pdTRUE, portMAX_DELAY);
            ESP_LOGW(TAG_MAIN, "Restarting after format...");
            esp_restart();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    } while (gpio_get_level(RBF_GPIO) == LOW); // While not armed

    data_g.status |= ARMED;

    eskf_var_t var = {
        .acc = 0.25f,
        .bar = 1.65f,
        .gps_h = 625.0f,
        .gps_vz = 0.0025f,
        .ba = 1e-4f,
        .bb = 5e-1f,
        .θe = 1e-6f,
    };

    eskf_config_t cfg = {
        .var = var,
        .dt = ICM_SAMPLE_RATE_S,
        .g = G,
        .igt = 3.0f,
        .idle_samples = 1000, // 10 seconds at 100 Hz
    };

    data_g.kf.cfg = cfg;
    eskf_init(&data_g.kf);

    // Create tasks
    xTaskCreatePinnedToCore(gps_task, "GPS", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(bmp_task, "BMP", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(fusion_task, "ICM", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_acquire, "ACQUIRE", configMINIMAL_STACK_SIZE * 4, NULL, 4, &xTaskAcquire, 0);
    xTaskCreatePinnedToCore(task_sd, "SD", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_littlefs, "LITTLEFS", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_buzzer_led, "BUZZER LED", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task_lora, "LORA", configMINIMAL_STACK_SIZE * 2, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_adc, "ADC", configMINIMAL_STACK_SIZE * 2, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(save_nvs_counters, "NVS COUNTER", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL, 0);
    #if CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_DEBUG
        xTaskCreatePinnedToCore(task_log, "LOG", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL, 0);
    #endif
}

void task_log(void *pvParameters)
{
    data_t data;
    while (true)
    {
        portENTER_CRITICAL(&xDATAMutex);
        data = data_g;
        portEXIT_CRITICAL(&xDATAMutex);

        ESP_LOGD("LOG", 
            "\n\tTime (ms):\t\t%lu\r\n"
            "\tStatus:\t\t\t%s\r\n"
            "\tVoltage (V):\t\t%.1f\r\n"
            "\tBMP pressure (Pa):\t%.2f\r\n"
            "\tBMP altitude (m):\t%.2f\r\n"
            "\tAccel (g):\t\tax: %.2f, ay: %.2f, az: %.2f\r\n"
            "\tGyro (°/s):\t\tgx: %.2f, gy: %.2f, gz: %.2f\r\n"
            "\tMag (µT):\t\tmx: %.2f, my: %.2f, mz: %.2f\r\n"
            "\t|Accel| (g):\t\t%.1f\r\n"
            "\tGPS coord (°):\t\tLat: %.5f, Lon: %.5f\r\n"
            "\tGPS altitude (m):\t%.2f\r\n"
            "\tGPS velZ (m/s):\t\t%.2f\r\n"
            "\tGPS sAcc (m/s):\t\t%.1f\r\n"
            "\tGPS fix:\t\t%d\r\n"
            "\tOrientation\t\tq1: %.5f, q2: %.5f, q3: %.5f, q4: %.5f\r\n"
            "\tKalman alt (m):\t\t%.2f\r\n"
            "\tKalman velZ (m/s):\t%.2f\r\n"
            "\tKalman apogee (m):\t%.2f\r\n"
            "\tKalman ba (m/s²):\t%.5f\r\n"
            "\tKalman bb (m):\t\t%.5f\r\n"
            "\tKalman θe (rad):\t%.5f\r\n"
            "-------------------------------------------------------------------------------------",
            data.time,
            uint8_to_binary(data.status),
            data.voltage*0.1f,
            data.bmp.pressure,
            data.bmp.altitude,
            data.icm.accel_x*ACC_SCALE, data.icm.accel_y*ACC_SCALE, data.icm.accel_z*ACC_SCALE,
            data.icm.gyro_x*GYRO_SCALE, data.icm.gyro_y*GYRO_SCALE, data.icm.gyro_z*GYRO_SCALE,
            data.icm.mag_x*MAG_SCALE, data.icm.mag_y*MAG_SCALE, data.icm.mag_z*MAG_SCALE,
            data.icm.accel*0.1f,
            data.gps.latitude, data.gps.longitude,
            data.gps.altitude,
            data.gps.vel_vertical,
            data.gps.sAcc*0.1f,
            data.gps.fix,
            data.icm.q1, data.icm.q2, data.icm.q3, data.icm.q4,
            data.kf.x.h,
            data.kf.x.vz,
            data.kf.x.apogee,
            data.kf.x.ba,
            data.kf.x.bb,
            data.kf.x.θe
        );
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}