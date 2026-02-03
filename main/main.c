#include "header.h"

#define HIGH 1
#define LOW 0

static const char *TAG_MAIN = "MAIN";
static const char *TAG_DEPLOY = "DEPLOY";

// task_deploy deploys parachutes
void task_deploy(void *pvParameters)
{
    gpio_set_direction(DROGUE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MAIN_GPIO, GPIO_MODE_OUTPUT);

    float current_altitude = 0;
    float max_altitude = 0;
    float start_altitude = 0;

    xQueueReceive(xAltQueue, &current_altitude, portMAX_DELAY);
    start_altitude = current_altitude;

    bool drogue_caindo = false;
    bool main_caindo = false;
    uint32_t local_status;

    while (true)
    {
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        local_status = STATUS;
        xSemaphoreGive(xStatusMutex);

        // Get current altitude
        xQueueReceive(xAltQueue, &current_altitude, portMAX_DELAY);

        // Check disarm condition
        if (!(local_status & ARMED))
        {
            // Terminates the task if the system is not armed
            ESP_LOGE(TAG_DEPLOY, "System disarmed. Terminating deploy task.");

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
                vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(LED_GPIO, LOW);
                gpio_set_level(BUZZER_GPIO, LOW);
                vTaskDelay(pdMS_TO_TICKS(950));
            }
            
            vTaskDelete(NULL);
        }
        
        // Update max altitude
        if (current_altitude > max_altitude) max_altitude = current_altitude;

        drogue_caindo = !(local_status & DROGUE_DEPLOYED) && (current_altitude < max_altitude - DROGUE_THRESHOLD);
        main_caindo = (local_status & DROGUE_DEPLOYED) && !(local_status & MAIN_DEPLOYED) && (current_altitude < start_altitude + MAIN_ALTITUDE);

        if (drogue_caindo)
        {
            for (int i = 0; i < 5; i++)
            {
                xQueueReceive(xAltQueue, &current_altitude, portMAX_DELAY);
                if (!(current_altitude < max_altitude - DROGUE_THRESHOLD))
                {
                    drogue_caindo = false;
                    break;
                }
            }

            if (!drogue_caindo)
                continue;

            gpio_set_level(DROGUE_GPIO, HIGH);
            ESP_LOGW(TAG_DEPLOY, "Drogue deployed");
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(DROGUE_GPIO, LOW);
            xSemaphoreTake(xStatusMutex, portMAX_DELAY);
            STATUS |= DROGUE_DEPLOYED;
            xSemaphoreGive(xStatusMutex);
        }

        else if (main_caindo)
        {
            for (int i = 0; i < 5; i++)
            {
                xQueueReceive(xAltQueue, &current_altitude, portMAX_DELAY);
                if (!(current_altitude < start_altitude + MAIN_ALTITUDE))
                {
                    main_caindo = false;
                    break;
                }
            }
            
            if (!main_caindo)
                continue;
            
            gpio_set_level(MAIN_GPIO, HIGH);
            ESP_LOGW(TAG_DEPLOY, "Main deployed");
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(MAIN_GPIO, LOW);
            xSemaphoreTake(xStatusMutex, portMAX_DELAY);
            STATUS |= MAIN_DEPLOYED;
            xSemaphoreGive(xStatusMutex);

            // Delete task
            vTaskDelete(NULL);
        }
    }
}

// task_buzzer_led blinks LED and beeps buzzer to indicate status
void task_buzzer_led(void *pvParameters)
{
    int32_t status_local;
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); 
                                         
        // Use local copy of STATUS because of delays
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        status_local = STATUS;
        xSemaphoreGive(xStatusMutex);

        // If LANDED, blink LED every second
        if (status_local & LANDED)
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
    // GPIO Initialization
    gpio_set_direction(RBF_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RBF_GPIO, GPIO_PULLUP_ONLY);

    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    // When initializing, blink LED and beep buzzer 3 times
    for (uint32_t i = 0; i < 3; ++i)
    {
        gpio_set_level(LED_GPIO, HIGH);
        gpio_set_level(BUZZER_GPIO, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(LED_GPIO, LOW);
        gpio_set_level(BUZZER_GPIO, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI("Buzzer LED", "Initialized");
}

static bool check_for_format_mode(void)
{
    if (gpio_get_level(BUTTON_GPIO) == LOW)
    {
        uint64_t time = esp_timer_get_time();
        while (gpio_get_level(BUTTON_GPIO) == LOW)
        {
            if (esp_timer_get_time() - time > 5000000)
            {
                ESP_LOGW("RESET", "Button pressed for 5 seconds. Formatting...");
                return true;
            }
            vTaskDelay(10);
        }
    }
    return false;
}

static void manage_nvs_counters(bool format_mode, file_counter_t *sd_counter, file_counter_t *lfs_counter)
{
    // Initialize NVS to store file counters
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Open NVS
    ESP_LOGI("NVS", "Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t nvs_handle;
    nvs_open("storage", NVS_READWRITE, &nvs_handle);
    int32_t sd_num = 0;
    int32_t lfs_num = 0;
    nvs_get_i32(nvs_handle, "sd_counter", &sd_num);
    nvs_get_i32(nvs_handle, "lfs_counter", &lfs_num);

    // Increment file counters
    if (sd_num < MAX_SD_FILES) sd_num++;
    else sd_num = 0;
    if (lfs_num < MAX_LFS_FILES) lfs_num++;
    else lfs_num = 0;

    if (format_mode)
    {
        sd_num = 0;
        lfs_num = 0;
    }

    // Save file counters
    nvs_set_i32(nvs_handle, "sd_counter", sd_num);
    nvs_set_i32(nvs_handle, "lfs_counter", lfs_num);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    // Populate the counter structs to be used by tasks
    sd_counter->file_num = sd_num;
    sd_counter->format = format_mode;
    lfs_counter->file_num = lfs_num;
    lfs_counter->format = format_mode;
}

void app_main(void)
{
    setup_peripherals();
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    bool format_mode = check_for_format_mode();

    // Create Mutexes
    xStatusMutex = xSemaphoreCreateMutex();
    // Create Queues
    static const int alt_queue_size = 30;
    static const int sd_queue_size = 10;
    static const int littlefs_queue_size = 10;
    static const int lora_queue_size = 15;
    xAltQueue = xQueueCreate(alt_queue_size, sizeof(float));
    xSDQueue = xQueueCreate(sd_queue_size, sizeof(data_t));
    xLittleFSQueue = xQueueCreate(littlefs_queue_size, sizeof(data_t));
    xLoraQueue = xQueueCreate(lora_queue_size, sizeof(data_t));

    file_counter_t counter_sd, counter_lfs;
    manage_nvs_counters(format_mode, &counter_sd, &counter_lfs);

    // If format is true, format SD and LittleFS, then restart
    if (format_mode)
    {
        xTaskCreate(task_sd, "SD", configMINIMAL_STACK_SIZE * 8, &counter_sd, 5, NULL);
        xTaskCreate(task_littlefs, "LittleFS", configMINIMAL_STACK_SIZE * 8, &counter_lfs, 5, NULL);
        vTaskDelay(pdMS_TO_TICKS(30000));
        esp_restart();
    }

    // If RBF is off at startup, set SAFE_MODE
    if (gpio_get_level(RBF_GPIO) == LOW)
    {
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        STATUS |= SAFE_MODE;
        xSemaphoreGive(xStatusMutex);
        ESP_LOGE("MAIN", "SAFE MODE");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    // Start tasks
    xTaskCreate(task_acquire, "Acquire", configMINIMAL_STACK_SIZE * 4, NULL, 4, NULL);
    xTaskCreate(task_sd, "SD", configMINIMAL_STACK_SIZE * 4, &counter_sd, 5, NULL);
    xTaskCreate(task_littlefs, "LittleFS", configMINIMAL_STACK_SIZE * 4, &counter_lfs, 5, NULL);
    xTaskCreate(task_buzzer_led, "Buzzer LED", configMINIMAL_STACK_SIZE * 2, NULL, 3, NULL);
    xTaskCreate(task_lora, "LoRa", configMINIMAL_STACK_SIZE * 4, NULL, 5, &xTaskLora);

    bool arm = false;
    bool disarm = false;
    uint32_t local_status;

    while (true)
    {
        // Logic for arming parachute deployment
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        local_status = STATUS;
        xSemaphoreGive(xStatusMutex);
        
        // If not armed, not in safe mode and RBF is off, arm the system
        arm = !(local_status & ARMED) && !(local_status & SAFE_MODE) && (gpio_get_level(RBF_GPIO) == LOW);
        // If already armed, check disarm condition
        disarm = !arm && !(local_status & FLYING) && (gpio_get_level(RBF_GPIO) == HIGH);

        if (arm)
        {
            xTaskCreate(task_deploy, "Deploy", configMINIMAL_STACK_SIZE * 2, NULL, 5, NULL); // Start deploy task
            xSemaphoreTake(xStatusMutex, portMAX_DELAY);
            STATUS |= ARMED;
            xSemaphoreGive(xStatusMutex);
            arm = false;
            ESP_LOGW(TAG_MAIN, "System ARMED.");
            for (uint32_t i = 0; i < 3; i++)
            {
                gpio_set_level(LED_GPIO, HIGH);
                gpio_set_level(BUZZER_GPIO, HIGH);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(LED_GPIO, LOW);
                gpio_set_level(BUZZER_GPIO, LOW);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        else if (disarm)
        {
            xSemaphoreTake(xStatusMutex, portMAX_DELAY);
            STATUS &= ~(ARMED);
            xSemaphoreGive(xStatusMutex);
            disarm = false;
            ESP_LOGI(TAG_MAIN, "Disarming system. Signaling...");
            gpio_set_level(LED_GPIO, HIGH);
            gpio_set_level(BUZZER_GPIO, HIGH);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(LED_GPIO, LOW);
            gpio_set_level(BUZZER_GPIO, LOW);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}