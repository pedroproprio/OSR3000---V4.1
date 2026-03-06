#include "header.h"

// TAGS
static const char *TAG_LITTLEFS = "LittleFS";
static const char *TAG_SD = "SD Card";
static const char *TAG_LORA = "LoRa";

void save_nvs_counters(void *pvParameters)
{
    const EventBits_t bits_to_wait = EVT_SD_DONE | EVT_LFS_DONE;

    // Wait until both SD and LittleFS tasks signal that they have finished
    xEventGroupWaitBits(xNVSCounterEvent, bits_to_wait, pdTRUE, pdTRUE, portMAX_DELAY);

    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));

    // Increment counters
    file_counter_g.file_numSD += 1;
    file_counter_g.file_numLFS += 1;
    
    // Save updated counters to NVS
    nvs_set_i32(nvs_handle, "sd_counter", file_counter_g.file_numSD);
    nvs_set_i32(nvs_handle, "lfs_counter", file_counter_g.file_numLFS);
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));

    nvs_close(nvs_handle);

    ESP_LOGW("NVS COUNTER", "Flight safely committed to NVS");

    vTaskDelete(NULL);
}


// task_sd reads data from queue and writes it to SD card
void task_sd(void *pvParameters)
{
    const file_counter_t counter = *(file_counter_t *)pvParameters;
    esp_err_t errSD;
    sdmmc_card_t *card;

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
        .max_transfer_sz = SD_BUFFER_SIZE,
    };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    // SPI initializer
    ESP_LOGD(TAG_SD, "Using SPI peripheral");
    errSD = spi_bus_initialize(host.slot, &bus_config, SDSPI_DEFAULT_DMA);
    if (errSD != ESP_OK)
    {
        ESP_LOGE(TAG_SD, "Failed to initialize SPI bus: %s", esp_err_to_name(errSD));
        vTaskDelete(NULL);
    }
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SS;
    slot_config.host_id = host.slot;
    ESP_LOGD(TAG_SD, "SPI bus initialized");

    // Mount filesystem
    ESP_LOGD(TAG_SD, "Mounting filesystem");
    errSD = esp_vfs_fat_sdspi_mount(SD_MOUNT, &host, &slot_config, &mount_config, &card);
    if (errSD != ESP_OK)
    {
        if (errSD == ESP_FAIL)
            ESP_LOGE(TAG_SD, "Failed to mount filesystem. "
                             "If you want the card to be formatted, set the CONFIG_SD_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        else
            ESP_LOGE(TAG_SD, "Failed to initialize the card: %s. ", esp_err_to_name(errSD));
        spi_bus_free(host.slot);
        ESP_LOGD(TAG_SD, "SPI bus freed");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG_SD, "Filesystem mounted");

    // Format mode
    if (counter.format == pdTRUE)
    {
        ESP_LOGW(TAG_SD, "Format mode enabled, formatting SD card");
        errSD = esp_vfs_fat_sdcard_format(SD_MOUNT, card);
        if (errSD != ESP_OK)
            ESP_LOGE(TAG_SD, "Failed to format FATFS: %s", esp_err_to_name(errSD));
        else
            ESP_LOGI(TAG_SD, "Format Successful");
        
        esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
        ESP_LOGI(TAG_SD, "Card unmounted");
        spi_bus_free(host.slot);
        ESP_LOGI(TAG_SD, "SPI bus freed");
        xEventGroupSetBits(xFormatEvent, EVT_SD_DONE);
        vTaskDelete(NULL);
    }

    // Print sd card info
    sdmmc_card_print_info(stdout, card);

    // Create log file
    char log_name[FILENAME_LENGTH];
    snprintf(log_name, FILENAME_LENGTH, "%s/flight%ld.bin", SD_MOUNT, counter.file_numSD);
    ESP_LOGI(TAG_SD, "Creating file %s", log_name);
    
    FILE *f = fopen(log_name, "wb");
    if (!f)
    {
        ESP_LOGE(TAG_SD, "Failed to open file for writing");
        esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
        ESP_LOGI(TAG_SD, "Card unmounted");
        spi_bus_free(host.slot);
        ESP_LOGI(TAG_SD, "SPI bus freed");
        vTaskDelete(NULL);
    }

    static uint8_t write_buffer[SD_BUFFER_SIZE];
    uint16_t buffer_offset = 0;
    save_t save_data;
    TickType_t last_sync = xTaskGetTickCount();

    while (true)
    {
        // Read data from queue
        if (xQueueReceive(xSDQueue, &save_data, portMAX_DELAY) == pdTRUE)
        {
            // If buffer is full, write to file
            if (buffer_offset + sizeof(save_t) > SD_BUFFER_SIZE)
            {
                size_t w = fwrite(write_buffer, 1, SD_BUFFER_SIZE, f);
                if (w != SD_BUFFER_SIZE)
                    ESP_LOGE(TAG_SD, "Failed to write data to file");
                else
                    ESP_LOGD(TAG_SD, "Data written to SD card");
                buffer_offset = 0; // Reset buffer index for next batch
                taskYIELD(); // Yield to allow other tasks to run
                
                if (xTaskGetTickCount() - last_sync >= pdMS_TO_TICKS(5000)) // Flushes every 5s
                {
                    fflush(f);
                    fsync(fileno(f));
                    last_sync = xTaskGetTickCount();
                    ESP_LOGD(TAG_SD, "File flushed");
                    taskYIELD(); // Yield to allow other tasks to run
                }
            }
            // Copy remaining data to buffer
            memcpy(&write_buffer[buffer_offset], &save_data, sizeof(save_t));
            buffer_offset += sizeof(save_t);
        }

        // Check if landing
        portENTER_CRITICAL(&xDATAMutex);
        bool landing = (data_g.status & LANDING);
        portEXIT_CRITICAL(&xDATAMutex);
        if (landing)
            break;
    }

    if(buffer_offset > 0) // Write remaining data to file
    {
        size_t w = fwrite(write_buffer, 1, buffer_offset, f);
        if (w != buffer_offset)
            ESP_LOGE(TAG_SD, "Failed to write remaining data to file");
        else
            ESP_LOGD(TAG_SD, "Remaining data written to SD card");
    }

    while(xQueueReceive(xB4LaunchQueue, &save_data, 0) == pdTRUE) // Write queue data to file
    {
        size_t w = fwrite(&save_data, 1, sizeof(save_t), f);
        if (w != sizeof(save_t))
            ESP_LOGE(TAG_SD, "Failed to write before launch data to file");
        else
            ESP_LOGD(TAG_SD, "Before launch data written to SD card");
    }
    
    ESP_LOGW(TAG_SD, "Landed, closing file and unmounting SD card");
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    taskYIELD(); // Yield to allow other tasks to run
    vTaskDelay(pdMS_TO_TICKS(20)); // Short delay to ensure SPI driver is done
    ESP_LOGI(TAG_SD, "File closed");
    esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
    ESP_LOGI(TAG_SD, "Card unmounted");
    spi_bus_free(host.slot);
    ESP_LOGI(TAG_SD, "SPI bus freed");
    xEventGroupSetBits(xNVSCounterEvent, EVT_SD_DONE); // Signal that SD task is done

    vTaskDelete(NULL);
}

// task_littlefs reads data from queue and writes it to LittleFS
void task_littlefs(void *pvParameters)
{
    const file_counter_t counter = *(file_counter_t *)pvParameters;
    esp_err_t errFS;

    // Settings for initializing LittleFS
    esp_vfs_littlefs_conf_t littlefs_config = {
        .base_path = "/littlefs",
        .partition_label = "littlefs",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    ESP_LOGI(TAG_LITTLEFS, "Initializing LittleFS");
    errFS = esp_vfs_littlefs_register(&littlefs_config);
    if (errFS != ESP_OK)
    {
        if (errFS == ESP_FAIL)
            ESP_LOGE(TAG_LITTLEFS, "Failed to mount or format filesystem");
        else if (errFS == ESP_ERR_NOT_FOUND)
            ESP_LOGE(TAG_LITTLEFS, "Failed to find LittleFS partition");
        else
            ESP_LOGE(TAG_LITTLEFS, "Failed to initialize LittleFS: %s", esp_err_to_name(errFS));

        vTaskDelete(NULL);
    }

    // Format mode
    if (counter.format == pdTRUE)
    {
        ESP_LOGW(TAG_LITTLEFS, "Format mode enabled, formatting LittleFS");
        errFS = esp_littlefs_format(littlefs_config.partition_label);
        if (errFS != ESP_OK)
            ESP_LOGE(TAG_LITTLEFS, "Failed to format LittleFS: %s", esp_err_to_name(errFS));
        else
            ESP_LOGI(TAG_LITTLEFS, "Format Successful");

        xEventGroupSetBits(xFormatEvent, EVT_LFS_DONE);
        vTaskDelete(NULL);
    }

    // Create log file
    char log_name[FILENAME_LENGTH];
    snprintf(log_name, FILENAME_LENGTH, "%s/flight%ld.bin", littlefs_config.base_path, counter.file_numLFS);
    ESP_LOGI(TAG_LITTLEFS, "Created file %s", log_name);

    FILE *f = fopen(log_name, "wb");
    if (!f)
    {
        ESP_LOGE(TAG_LITTLEFS, "Failed to open file for writing");
        esp_vfs_littlefs_unregister(littlefs_config.partition_label);
        ESP_LOGI(TAG_LITTLEFS, "LittleFS unmounted");
        vTaskDelete(NULL);
    }

    static uint8_t buffer[LITTLEFS_BUFFER_SIZE];
    uint16_t buffer_offset = 0;
    save_t save_data;
    bool _lfs_full;

    size_t total = 0, used = 0;
    errFS = esp_littlefs_info(littlefs_config.partition_label, &total, &used);
    if (errFS != ESP_OK)
        ESP_LOGE(TAG_LITTLEFS, "Failed to get LittleFS partition information: %s", esp_err_to_name(errFS));
    else
        ESP_LOGI(TAG_LITTLEFS, "Partition size: %d/%d (%.2f%%)", used, total, (float)(used / total * 100.0f));

    while (true)
    {
        _lfs_full = atomic_load_explicit(&lfs_full, memory_order_relaxed);
        // Read data from queue
        if (xQueueReceive(xLittleFSQueue, &save_data, portMAX_DELAY) == pdTRUE)
        {
            // If buffer is full, write to file
            if (buffer_offset + sizeof(save_t) > LITTLEFS_BUFFER_SIZE)
            {
                if (!_lfs_full)
                {
                    if (used + buffer_offset > MAX_FLASH_SIZE_USED * total) // Check if there's space before writing
                    {
                        ESP_LOGW(TAG_LITTLEFS, "Flash memory almost full.");
                        _lfs_full = true;
                        atomic_store_explicit(&lfs_full, true, memory_order_relaxed);
                    }
                    else
                    {
                        size_t w = fwrite(buffer, 1, LITTLEFS_BUFFER_SIZE, f);
                        if (w != LITTLEFS_BUFFER_SIZE)
                            ESP_LOGE(TAG_LITTLEFS, "Failed to write data to file");
                        else
                            ESP_LOGD(TAG_LITTLEFS, "Data written to LittleFS");
                        used += sizeof(buffer); // Update used space tracker
                        taskYIELD(); // Yield to allow other tasks to run
                    }
                }
                buffer_offset = 0; // Reset buffer index for next batch (or drop if full)
            }
            // Copy remaining data to buffer if we still have space available
            if (!_lfs_full)
            {
                memcpy(&buffer[buffer_offset], &save_data, sizeof(save_t));
                buffer_offset += sizeof(save_t);
            }
        }

        // Check if landed
        portENTER_CRITICAL(&xDATAMutex);
        bool landed = (data_g.status & LANDED);
        portEXIT_CRITICAL(&xDATAMutex);
        if (landed)
            break;
    }

    if (buffer_offset > 0) // Write remaining data to file
    {
        if (_lfs_full || used + buffer_offset > MAX_FLASH_SIZE_USED * total) // Check if there's space before writing
            ESP_LOGW(TAG_LITTLEFS, "Flash memory almost full. Remaining data not written.");
        else
        {
            size_t w = fwrite(buffer, 1, buffer_offset, f);
            if (w != buffer_offset)
                ESP_LOGE(TAG_LITTLEFS, "Failed to write remaining data to file");
            else
                ESP_LOGD(TAG_LITTLEFS, "Remaining data written to LittleFS");
            used += buffer_offset; // Update used space tracker
        }
    }
    
    ESP_LOGW(TAG_LITTLEFS, "Landed, closing file and unmounting LittleFS");
    fclose(f);
    ESP_LOGI(TAG_LITTLEFS, "File closed");
    vTaskDelay(pdMS_TO_TICKS(20));
    esp_vfs_littlefs_unregister(littlefs_config.partition_label);
    ESP_LOGI(TAG_LITTLEFS, "LittleFS unmounted");
    xEventGroupSetBits(xNVSCounterEvent, EVT_LFS_DONE); // Signal that LFS task is done
    
    vTaskDelete(NULL);
}

static SemaphoreHandle_t xLoraAuxSem = NULL;

static void IRAM_ATTR handle_interrupt_fromisr(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xLoraAuxSem, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
}

static bool lora_wait_aux_high(TickType_t timeout_ticks)
{
    if (gpio_get_level(LORA_AUX))
        return true;

    if (xSemaphoreTake(xLoraAuxSem, timeout_ticks) == pdTRUE)
        return true;

    ESP_LOGW(TAG_LORA, "AUX timeout");
    return false;
}

static bool lora_send_packet(const send_t *pkt)
{
    if (pkt == NULL)
        return false;

    const uint8_t *buf = (const uint8_t *)pkt;
    const size_t total = sizeof(send_t);

    if (!lora_wait_aux_high(pdMS_TO_TICKS(CONFIG_E220_AUX_TIMEOUT_MS))) // Waits for LoRa to be ready
    {
        ESP_LOGW(TAG_LORA, "LoRa busy before TX");
        return false;
    }

    int written = uart_write_bytes(LORA_UART_NUM, (const char *)buf, total); // Writes packet to UART at once

    if (written != total)
    {
        ESP_LOGE(TAG_LORA, "UART write failed (%d/%d)", written, total);
        return false;
    }

    if (!lora_wait_aux_high(pdMS_TO_TICKS(CONFIG_E220_AUX_TIMEOUT_MS))) // Waits for LoRa to finish transmission
    {
        ESP_LOGW(TAG_LORA, "TX not confirmed by AUX");
        return false;
    }

    return true;
}

static void e220_set_config(void) // E220-900T22D
{
    const uint8_t config_cmd[] = {
        0xC2,   // temporary register
        0x00,   // starting address
        0x08,   // length
        0xFF,   // ADDH
        0xFF,   // ADDL, no address filtering
        0xE0,   // REG0 (0b11100000: 115200 baud, 8N1, 2.4k ADR)
        0xC0,   // REG1 (0b11000000: 32 bytes sub-packet, disable RSSI Ambient noise, 22dBm)
        0x41,   // REG2 (850.125 + CH*1M = 915.125Mhz)
        0x00,   // REG3 (0b00000000: disable RSSI byte, transparent transmission mode, disable LBT, WOR cycle not applicable)
        0x00,   // CRYPT_H (encryption key MSB)
        0x00,   // CRYPT_L (encryption key LSB)
    };

    uart_flush(LORA_UART_NUM); // Flush UART to clear any residual data
    xSemaphoreTake(xLoraAuxSem, pdMS_TO_TICKS(200));
    uart_write_bytes(LORA_UART_NUM, (const char *)config_cmd, sizeof(config_cmd));
    xSemaphoreTake(xLoraAuxSem, pdMS_TO_TICKS(200));

    uint8_t response[sizeof(config_cmd)];
    uart_read_bytes(LORA_UART_NUM, response, sizeof(config_cmd), pdMS_TO_TICKS(100));
    if (response[0] != 0xC1)
        ESP_LOGE(TAG_LORA, "Failed to set LoRa configuration, response 0x%02X", response[0]);
    
    ESP_ERROR_CHECK(uart_set_baudrate(LORA_UART_NUM, LORA_BAUDRATE)); // Update baudrate after configuration
    // 8N1 is already set in uart_config
}

static void lora_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 9600, // E220 default baudrate is 9600
        .data_bits = UART_DATA_8_BITS, // 8
        .parity = UART_PARITY_DISABLE, // N
        .stop_bits = UART_STOP_BITS_1, // 1
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(LORA_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LORA_UART_NUM, LORA_TX, LORA_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(LORA_UART_NUM, 2048, 2048, 0, NULL, 0));

    xLoraAuxSem = xSemaphoreCreateBinary();

    gpio_set_direction(LORA_AUX, GPIO_MODE_INPUT);
    gpio_pullup_en(LORA_AUX); // Enable pull-up on AUX pin
    gpio_set_intr_type(LORA_AUX, GPIO_INTR_POSEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(LORA_AUX, handle_interrupt_fromisr, NULL);

    // Set M0 and M1 to 1 for configuration mode
    gpio_set_direction(LORA_M0, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_M1, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_M0, 1);
    gpio_set_level(LORA_M1, 1);
    vTaskDelay(pdMS_TO_TICKS(50)); // Short delay

    e220_set_config();

    // Set M0 and M1 to 0 for normal mode
    gpio_set_level(LORA_M0, 0);
    gpio_set_level(LORA_M1, 0);
    vTaskDelay(pdMS_TO_TICKS(50)); // Short delay

    ESP_LOGI(TAG_LORA, "LoRa UART initialized (baud %d)", LORA_BAUDRATE);
}

void task_lora(void *pvParameters)
{
    lora_init();

    send_t send_data;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        xQueuePeek(xLoraQueue, &send_data, 0); // Non-blocking peek, will use last data if queue hasn't been updated yet

        if (!lora_send_packet(&send_data))
            ESP_LOGW(TAG_LORA, "Failed to send LoRa packet — discarded");

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LORA_RATE_MS));
    }
}