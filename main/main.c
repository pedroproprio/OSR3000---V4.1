#include "header.h"

#define HIGH 1
#define LOW 0

static const char *TAG_MAIN = "MAIN";

// task_buzzer_led blinks LED and beeps buzzer to indicate status
void task_buzzer_led(void *pvParameters)
{
    uint8_t status;
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); 
                                         
        // Use local copy of STATUS because of delays
        portENTER_CRITICAL(&xDATAMutex);
        status = data_g.status;
        portEXIT_CRITICAL(&xDATAMutex);

        // If LANDED, blink LED every second
        if (status & LANDED)
        {
            while(true)
            {
                gpio_set_level(LED_GPIO, HIGH);
                gpio_set_level(BUZZER_GPIO, HIGH);
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level(LED_GPIO, LOW);
                gpio_set_level(BUZZER_GPIO, LOW);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
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

    // When initializing, blink LED and beep buzzer 3 times
    for (uint32_t i = 0; i < 3; ++i)
    {
        gpio_set_level(LED_GPIO, HIGH);
        gpio_set_level(BUZZER_GPIO, HIGH);
        vTaskDelay(pdMS_TO_TICKS(150));
        gpio_set_level(LED_GPIO, LOW);
        gpio_set_level(BUZZER_GPIO, LOW);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
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
                return true;
            }
            vTaskDelay(10);
        }
    }
    return false;
}

static void manage_nvs_counters(const bool format_mode)
{
    // Initialize NVS to store file counters
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_LOGW("NVS", "NVS partition was truncated, erasing...");
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

    bool format_mode = check_for_format_mode();
    manage_nvs_counters(format_mode);

    // Create Mutexes
    xI2CMutex = xSemaphoreCreateMutex();
    xNVSCounterEvent = xEventGroupCreate();

    // Create Queues
    xSDQueue = xQueueCreate(SD_QUEUE_SIZE, sizeof(save_t));
    xLittleFSQueue = xQueueCreate(LITTLEFS_QUEUE_SIZE, sizeof(save_t));
    xLoraQueue = xQueueCreate(LORA_QUEUE_SIZE, sizeof(send_t));

    // If format is true, format SD and LittleFS, then restart
    if (format_mode)
    {
        xTaskCreate(task_sd, "SD", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 5, NULL);
        xTaskCreate(task_littlefs, "LittleFS", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 5, NULL);
        vTaskDelay(pdMS_TO_TICKS(30000)); // Wait 30 seconds for formatting to complete
        ESP_LOGW(TAG_MAIN, "Restarting after format...");
        esp_restart();
    }

    // If RBF is off at startup, set SAFE_MODE
    if (gpio_get_level(RBF_GPIO) == LOW)
        data_g.status |= SAFE_MODE;

    // Create tasks
    xTaskCreatePinnedToCore(gps_task, "GPS", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(bmp_task, "BMP", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(fusion_task, "ICM", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_acquire, "ACQUIRE", configMINIMAL_STACK_SIZE * 4, NULL, 4, &xTaskAcquire, 0);
    xTaskCreatePinnedToCore(task_log, "LOG", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task_sd, "SD", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_littlefs, "LITTLEFS", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_buzzer_led, "BUZZER LED", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task_lora, "LORA", configMINIMAL_STACK_SIZE * 2, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_adc, "ADC", configMINIMAL_STACK_SIZE * 2, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(save_nvs_counters, "NVS COUNTER", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task_deploy, "DEPLOY", configMINIMAL_STACK_SIZE * 2, NULL, 5, &xTaskDeploy, 0);

    static bool arm = false, disarm = false;
    uint8_t status;

    // Logic for arming parachute deployment
    while (true)
    {
        portENTER_CRITICAL(&xDATAMutex);
        status = data_g.status;
        portEXIT_CRITICAL(&xDATAMutex);
        
        // If not armed, not in safe mode and RBF is off, arm the system
        arm = !(status & ARMED) && !(status & SAFE_MODE) && gpio_get_level(RBF_GPIO) == HIGH;
        // If already armed, check disarm condition
        disarm = !arm && !(status & FLYING) && (gpio_get_level(RBF_GPIO) == LOW);

        if (arm)
        {
            portENTER_CRITICAL(&xDATAMutex);
            data_g.status |= ARMED;
            portEXIT_CRITICAL(&xDATAMutex);

            xTaskNotifyGive(xTaskDeploy); // Notify deploy task to start checking for deployment conditions

            ESP_LOGW(TAG_MAIN, "System ARMED.");
            for (uint8_t i = 0; i < 3; i++)
            {
                gpio_set_level(LED_GPIO, HIGH);
                gpio_set_level(BUZZER_GPIO, HIGH);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(LED_GPIO, LOW);
                gpio_set_level(BUZZER_GPIO, LOW);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            arm = false;
        }
        else if (disarm)
        {
            portENTER_CRITICAL(&xDATAMutex);
            data_g.status &= ~(ARMED);
            portEXIT_CRITICAL(&xDATAMutex);
            
            ESP_LOGW(TAG_MAIN, "Disarming system. Signaling...");
            gpio_set_level(LED_GPIO, HIGH);
            gpio_set_level(BUZZER_GPIO, HIGH);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(LED_GPIO, LOW);
            gpio_set_level(BUZZER_GPIO, LOW);
            disarm = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}