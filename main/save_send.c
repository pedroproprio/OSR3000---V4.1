#include "header.h"

// TAGS
static const char *TAG_LITTLEFS = "LittleFS";
static const char *TAG_SD = "SD Card";
static const char *TAG_LORA = "LoRa";

// task_sd reads data from queue and writes it to SD card
void task_sd(void *pvParameters)
{
    esp_err_t errSD;
    sdmmc_card_t *card;
    file_counter_t counterSD = *(file_counter_t *)pvParameters;

    // Settings for mounting FAT filesystem
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = MAX_SD_FILES,
        .allocation_unit_size = SD_UNIT_SIZE,
    };
    // Set CONFIG_SD_FORMAT_IF_MOUNT_FAILED to TRUE or FALSE
    // When format_if_mount_failed is set to true, SD card will be partitioned and formatted
    
    ESP_LOGI(TAG_SD, "Initializing SD card");

    // Settings for initializing SPI bus
    spi_bus_config_t bus_config = {
        .mosi_io_num = MOSI,
        .miso_io_num = MISO,
        .sclk_io_num = SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SD_TRANSF_SIZE,
    };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    // SPI initializer
    ESP_LOGI(TAG_SD, "Using SPI peripheral");
    errSD = spi_bus_initialize(host.slot, &bus_config, SDSPI_DEFAULT_DMA);
    if (errSD != ESP_OK)
    {
        ESP_LOGE(TAG_SD, "Failed to initialize SPI bus.");
        vTaskDelete(NULL);
    }
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SS;
    slot_config.host_id = host.slot;
    ESP_LOGI(TAG_SD, "SPI bus initialized");

    // Mount filesystem
    ESP_LOGI(TAG_SD, "Mounting filesystem");
    errSD = esp_vfs_fat_sdspi_mount(SD_MOUNT, &host, &slot_config, &mount_config, &card);
    if (errSD != ESP_OK)
    {
        if (errSD == ESP_FAIL)
        {
            ESP_LOGE(TAG_SD, "Failed to mount filesystem. "
                             "If you want the card to be formatted, set the CONFIG_SD_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            ESP_LOGE(TAG_SD, "Failed to initialize the card (%s). ",
                     esp_err_to_name(errSD));
        }
        spi_bus_free(host.slot);
        ESP_LOGI(TAG_SD, "SPI bus freed");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG_SD, "Filesystem mounted");

    // Format mode
    if (counterSD.format == pdTRUE)
    {
        errSD = esp_vfs_fat_sdcard_format(SD_MOUNT, card);
        if (errSD != ESP_OK)
        {
            ESP_LOGE(TAG_SD, "Failed to format FATFS (%s)", esp_err_to_name(errSD));
        }
        else
            ESP_LOGI(TAG_SD, "Format Successful");
        
        esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
        ESP_LOGI(TAG_SD, "Card unmounted");
        spi_bus_free(host.slot);
        ESP_LOGI(TAG_SD, "SPI bus freed");        
        vTaskDelete(NULL);
    }

    // Print sd card info
    sdmmc_card_print_info(stdout, card);

    // Create log file
    char log_name[FILENAME_LENGTH];
    snprintf(log_name, FILENAME_LENGTH, "%s/flight%ld.bin", SD_MOUNT, counterSD.file_num);
    ESP_LOGI(TAG_SD, "Creating file %s", log_name);

    FILE *f = fopen(log_name, "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG_SD, "Failed to open file for writing");
        esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
        ESP_LOGI(TAG_SD, "Card unmounted");
        spi_bus_free(host.slot);
        ESP_LOGI(TAG_SD, "SPI bus freed");
        vTaskDelete(NULL);
    }
    fclose(f);

    while (true)
    {
        data_t buffer[SD_BUFFER_SIZE / sizeof(data_t)]; 

        // Read data from queue
        for (int i = 0; i < SD_BUFFER_SIZE / sizeof(data_t); ++i)
        {
            xQueueReceive(xSDQueue, &buffer[i], portMAX_DELAY);
        }

        // Write buffer to file
        f = fopen(log_name, "a");
        if (f == NULL)
        {
            ESP_LOGE(TAG_SD, "Failed to open file for writing");
        }
        fwrite(buffer, sizeof(data_t), SD_BUFFER_SIZE / sizeof(data_t), f);
        fclose(f);
        ESP_LOGI(TAG_SD, "Data written to SD card");

        // Check if landed
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        if (STATUS & LANDED)
        {
            xSemaphoreGive(xStatusMutex);

            ESP_LOGW(TAG_SD, "Landed, unmounting SD card");
            esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
            ESP_LOGI(TAG_SD, "Card unmounted");
            spi_bus_free(host.slot);
            ESP_LOGI(TAG_SD, "SPI bus freed");

            vTaskDelete(NULL);
        }
        else
            xSemaphoreGive(xStatusMutex);
    }
}

// task_littlefs reads data from queue and writes it to LittleFS
void task_littlefs(void *pvParameters)
{
    esp_err_t errFS;
    file_counter_t counterFS = *(file_counter_t *)pvParameters;

    // Settings for initializing LittleFS
    esp_vfs_littlefs_conf_t littlefs_config = {
        .base_path = "/littlefs",
        .partition_label = "littlefs",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    ESP_LOGW(TAG_LITTLEFS, "Initializing LittleFS");
    errFS = esp_vfs_littlefs_register(&littlefs_config);
    if (errFS != ESP_OK)
    {
        if (errFS == ESP_FAIL)
            ESP_LOGE(TAG_LITTLEFS, "Failed to mount or format filesystem");

        else if (errFS == ESP_ERR_NOT_FOUND)
            ESP_LOGE(TAG_LITTLEFS, "Failed to find LittleFS partition");

        else
            ESP_LOGE(TAG_LITTLEFS, "Failed to initialize LittleFS (%s)", esp_err_to_name(errFS));

        vTaskDelete(NULL);
    }

    size_t total = 0, used = 0;
    errFS = esp_littlefs_info(littlefs_config.partition_label, &total, &used);
    if (errFS != ESP_OK)
    {
        ESP_LOGE(TAG_LITTLEFS, "Failed to get LittleFS partition information (%s)", esp_err_to_name(errFS));
    }
    else
    {
        ESP_LOGW(TAG_LITTLEFS, "Partition size: total: %d, used: %d", total, used);
    }

    // Format mode
    if (counterFS.format == pdTRUE)
    {
        errFS = esp_littlefs_format(littlefs_config.partition_label);
        if (errFS != ESP_OK)
        {
            ESP_LOGE(TAG_LITTLEFS, "Failed to format LittleFS (%s)", esp_err_to_name(errFS));
        }
        else
            ESP_LOGI(TAG_LITTLEFS, "Format Successful");

        vTaskDelete(NULL);
    }

    // Create log file
    char log_name[FILENAME_LENGTH];
    snprintf(log_name, FILENAME_LENGTH, "%s/flight%ld.bin", littlefs_config.base_path, counterFS.file_num);
    ESP_LOGI(TAG_LITTLEFS, "Creating file %s", log_name);

    FILE *f = fopen(log_name, "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG_LITTLEFS, "Failed to open file for writing");
        esp_vfs_littlefs_unregister(littlefs_config.partition_label);
        ESP_LOGI(TAG_LITTLEFS, "LittleFS unmounted");
        vTaskDelete(NULL);
    }
    fclose(f);

    uint32_t oldest_file_num = counterFS.file_num;

    while (true)
    {
        data_t buffer[SD_BUFFER_SIZE / sizeof(data_t)];

        // Read data from queue
        for (int i = 0; i < SD_BUFFER_SIZE / sizeof(data_t); ++i)
        {
            xQueueReceive(xLittleFSQueue, &buffer[i], portMAX_DELAY);  
        }

        // Delete oldest file if disk space is full
        while (used + sizeof(buffer) > MAX_USED * total)
        {
            oldest_file_num++;
            if (oldest_file_num > MAX_LFS_FILES)
            {
                oldest_file_num = 0;
            }
            char oldest_file_name[FILENAME_LENGTH];
            snprintf(oldest_file_name, FILENAME_LENGTH, "%s/flight%ld.bin", littlefs_config.base_path, oldest_file_num);

            struct stat st;
            if (stat(oldest_file_name, &st) == 0) // If file exists
            {
                ESP_LOGW(TAG_LITTLEFS, "Deleting file %s", oldest_file_name);
                unlink(oldest_file_name); // Delete file
            }

            esp_littlefs_info(littlefs_config.partition_label, &total, &used); // Update used space

            if (oldest_file_num == counterFS.file_num) // If oldest file is current file
            {
                ESP_LOGE(TAG_LITTLEFS, "No more disk space, unmounting LittleFS");

                esp_vfs_littlefs_unregister(littlefs_config.partition_label);
                ESP_LOGI(TAG_LITTLEFS, "LittleFS unmounted");

                xSemaphoreTake(xStatusMutex, portMAX_DELAY);
                STATUS |= LFS_FULL;
                xSemaphoreGive(xStatusMutex);

                vTaskDelete(NULL);
            }
        }

        // Write buffer to file
        f = fopen(log_name, "a");
        if (f == NULL)
        {
            ESP_LOGE(TAG_LITTLEFS, "Failed to open file for writing");
        }
        fwrite(buffer, sizeof(data_t), SD_BUFFER_SIZE / sizeof(data_t), f);
        fclose(f);                             
        ESP_LOGI(TAG_LITTLEFS, "Data written to LittleFS.");

        // Update used space tracker
        used += sizeof(buffer);  

        // Check if landed
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        if (STATUS & LANDED)
        {
            xSemaphoreGive(xStatusMutex);
            ESP_LOGW(TAG_LITTLEFS, "Landed, unmounting LittleFS");
            esp_vfs_littlefs_unregister(littlefs_config.partition_label);
            ESP_LOGI(TAG_LITTLEFS, "LittleFS unmounted");
            vTaskDelete(NULL);
        }
        else
            xSemaphoreGive(xStatusMutex);
    }
}

static SemaphoreHandle_t xLoraAuxSem = NULL;

static void IRAM_ATTR handle_interrupt_fromisr(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xLoraAuxSem, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static bool lora_send_data_blocking(const data_t *p)
{
    const uint8_t *buf = (const uint8_t *)p;
    const int total = sizeof(data_t);
    int written = 0;
    int attempt = 0;

    // this will try to write all bytes
    while (written < total && attempt < LORA_TX_RETRIES) {
        int w = uart_write_bytes(UART_NUM_2, (const char *)(buf + written), total - written);
        if (w > 0) {
            written += w;
            attempt = 0;
            //ESP_LOGD(TAG_LORA, "uart_write_bytes wrote %d/%d", written, total);
        } else {
            ESP_LOGW(TAG_LORA, "uart_write_bytes returned %d, retrying...", w);
            attempt++;
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    if (written != total) {
        ESP_LOGE(TAG_LORA, "Failed to write full payload to UART (%d/%d)", written, total);
        return false;
    }

    // Wait for UART TX to complete
    esp_err_t werr = uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(CONFIG_E220_TX_DONE_TIMEOUT_MS));
    if (werr != ESP_OK) {
        ESP_LOGW(TAG_LORA, "uart_wait_tx_done returned %s", esp_err_to_name(werr));
        // we still wait for AUX below, but this indicates driver-level issues
    }

    // Wait for AUX to go HIGH again
    if (xLoraAuxSem != NULL) {
        if (xSemaphoreTake(xLoraAuxSem, pdMS_TO_TICKS(CONFIG_E220_AUX_TIMEOUT_MS)) == pdTRUE) {
            // AUX confirmed = success
            //ESP_LOGD(TAG_LORA, "AUX confirmed transmission finished");
            return true;
        } else {
            ESP_LOGW(TAG_LORA, "Timeout waiting for AUX after UART TX");
            return false;
        }
    } else {
        // if the semaphore is NULL, fallback to polling (should not happen)
        uint32_t start = esp_log_timestamp();
        while (gpio_get_level(LORA_AUX) == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            if ((esp_log_timestamp() - start) > CONFIG_E220_AUX_TIMEOUT_MS) {
                ESP_LOGW(TAG_LORA, "Timeout polling AUX after UART TX");
                return false;
            }
        }
        return true;
    }
}

void lora_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = LORA_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(LORA_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LORA_UART_NUM, LORA_TX, LORA_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(LORA_UART_NUM, 2048, 2048, 0, NULL, 0));

    xLoraAuxSem = xSemaphoreCreateBinary();

    gpio_set_direction(LORA_AUX, GPIO_MODE_INPUT);
    gpio_pullup_dis(LORA_AUX);
    gpio_set_intr_type(LORA_AUX, GPIO_INTR_POSEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(LORA_AUX, handle_interrupt_fromisr, NULL);

    ESP_LOGI(TAG_LORA, "LoRa UART initialized (baud %d)", LORA_BAUDRATE);
}

void task_lora(void *pvParameters)
{
    data_t data;

    lora_init();

    while (true)
    {
        while (xQueueReceive(xLoraQueue, &data, portMAX_DELAY) == pdTRUE)
        {
            // Clear any stale semaphore state
            xSemaphoreTake(xLoraAuxSem, 0);
            // Wait for AUX HIGH before transmission
            if (gpio_get_level(LORA_AUX) == 0) {
                // Wait for ISR to signal AUX rising edge
                if (xSemaphoreTake(xLoraAuxSem, pdMS_TO_TICKS(CONFIG_E220_AUX_TIMEOUT_MS)) != pdTRUE) {
                    ESP_LOGW(TAG_LORA, "Timeout waiting for AUX before TX");
                    continue; // Skip this transmission
                }
            }

            bool ok = lora_send_data_blocking(&data); 
            if (!ok) { 
                ESP_LOGE(TAG_LORA, "Failed to send LoRa packet — discarded"); 
            }
            // Wait for ISR to signal that AUX went HIGH again (finished TX)
            xSemaphoreTake(xLoraAuxSem, pdMS_TO_TICKS(1000));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}