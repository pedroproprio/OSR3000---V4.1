#include "header.h"

#define CALIBRATION_SAMPLES 20

static const char *TAG = "GPS";

static void ubx_parse_byte(uint8_t c, gps_sample_t *gps)
{
    static uint8_t state = 0;
    static uint8_t class, id;
    static uint16_t len, cnt;
    static uint8_t ck_a, ck_b;
    static uint8_t payload[64]; // NAV-VELNED has 36 bytes

    switch(state)
    {
        case 0: if(c == 0xB5) state = 1; break;
        case 1: state = (c == 0x62) ? 2 : 0; break;
        case 2: class=c; ck_a=c; ck_b=c; state=3; break;
        case 3: id=c; ck_a+=c; ck_b+=ck_a; state=4; break;
        case 4: len=c; ck_a+=c; ck_b+=ck_a; state=5; break;
        case 5: len|=(c<<8); ck_a+=c; ck_b+=ck_a; cnt=0; state=6; break;
        case 6: if(cnt < sizeof(payload)) payload[cnt] = c; ck_a+=c; ck_b+=ck_a; cnt++; if(cnt>=len) state=7; break;
        case 7: state = (c==ck_a) ? 8 : 0; break;
        case 8: state=0; if(c!=ck_b) break;
        
        // NAV-VELNED, supported on: u-blox 7 firmware version 1.00
        if(class==0x01 && id==0x12 && len>=36)
        {
            int32_t velD; // I4
            uint32_t sAcc; // U4
            memcpy(&velD, &payload[12], sizeof(int32_t)); // velD in cm/s (down positive)
            memcpy(&sAcc, &payload[28], sizeof(uint32_t)); // sAcc in cm/s
            if (sAcc > 2550) sAcc = 2550; // Constraint for uint8_t storage (255.0 m/s * 10)
            gps->vel_vertical = -(velD * 0.01f); // m/s (up positive)
            gps->sAcc = (uint8_t)roundf(sAcc * 0.1f); // m/s * 10
        }
        break;
    }
}

static void gps_enable_nav_velned(void)
{
    const uint8_t ubx_enable_nav_velned[] = { // supported on: u-blox 7 firmware version 1.00
        0xB5, 0x62,    // message header
        0x06, 0x01,    // CFG-MSG
        0x03, 0x00,    // payload length (3 bytes)
        0x01,          // msgClass (NAV)
        0x12,          // msgID (VELNED)
        0x01,          // rate (enable)
        0x1E, 0x67     // checksum
    };    
    
    uart_write_bytes(GPS_UART_NUM, (const char*)ubx_enable_nav_velned, sizeof(ubx_enable_nav_velned));
}    

static void gps_set_rate(void)
{
    const uint8_t ubx_set_rate[] = { // supported on: u-blox 7 firmware version 1.00
        0xB5, 0x62,                  // message header
        0x06, 0x08,                  // CFG-RATE
        0x06, 0x00,                  // payload length (6 bytes)
        GPS_SAMPLE_RATE_MS, 0x00,    // measRate MSB (0xC8, 0x00)
        0x01, 0x00,                  // navRate (1 cycle, more cycles only supported on: u-blox 8 / u-blox M8 from protocol version 15 up to version 23.01)
        0x01, 0x00,                  // timeRef (GPS)
        0xDE, 0x6A                   // checksum
    };    

    uart_write_bytes(GPS_UART_NUM, (const char*)ubx_set_rate, sizeof(ubx_set_rate));
}    

static void gps_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = GPS_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX, GPS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_BUFF_SIZE, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "GPS UART initialized (baud %d)", GPS_BAUDRATE);
    vTaskDelay(pdMS_TO_TICKS(200)); // Wait for GPS to stabilize

    gps_enable_nav_velned();
    vTaskDelay(pdMS_TO_TICKS(50)); // Short delay
    gps_set_rate(); // Could check ACK for both commands, for now it's assumed to work
}

void gps_task(void *pvParameters)
{
    gps_init();

    char nmea_line[120];
    int nmea_pos = 0;

    uint8_t rx_buffer[GPS_RX_CHUNK];
    static float initial_altitude = 0;
    gps_sample_t gps = {0};

    bool logged_first_sentence = false;
    int last_fix_quality = -1;
    TickType_t last_no_fix_log = xTaskGetTickCount();

    // Calibration samples 
    float altitude_sum = 0.0f;
    uint8_t altitude_samples = 0;

    while (true)
    {
        // Read bytes from UART in chunks. The parser can handle partial data. Timeout of 20ms (max 50hz).
        int rx_len = uart_read_bytes(GPS_UART_NUM, rx_buffer, GPS_RX_CHUNK, pdMS_TO_TICKS(20));
        if (rx_len > 0)
        {
            for (int i = 0; i < rx_len; ++i)
            {
                uint8_t c = rx_buffer[i];

                /* ---------- UBX PARSER ---------- */
                ubx_parse_byte(c, &gps);

                /* ---------- NMEA PARSER ---------- */
                if (c == '$')
                {
                    nmea_pos = 0;
                    nmea_line[nmea_pos++] = c;
                }
                else if (nmea_pos > 0)
                {
                    if (c == '\n')
                    {
                        nmea_line[nmea_pos] = '\0';
                        enum minmea_sentence_id sid = minmea_sentence_id(nmea_line, false);
                        if (!logged_first_sentence && sid != MINMEA_INVALID && sid != MINMEA_UNKNOWN)
                        {
                            ESP_LOGI(TAG, "First valid NMEA sentence received: %s", minmea_sentence(sid));
                            logged_first_sentence = true;
                        }

                        switch (sid)
                        {
                            case MINMEA_SENTENCE_GGA:
                            {
                                struct minmea_sentence_gga gga;
                                if (minmea_check(nmea_line, false) && minmea_parse_gga(&gga, nmea_line))
                                {
                                    if (gga.fix_quality != last_fix_quality)
                                    {
                                        ESP_LOGI(TAG, "GGA fix_quality changed: %d -> %d", last_fix_quality, gga.fix_quality);
                                        last_fix_quality = gga.fix_quality;
                                        gps.fix = (uint8_t)gga.fix_quality;
                                    }
                                    
                                    if (gga.fix_quality == 0)
                                    {
                                        TickType_t now = xTaskGetTickCount();
                                        if ((now - last_no_fix_log) >= pdMS_TO_TICKS(30000))
                                        {
                                            ESP_LOGW(TAG, "No fix yet: sats=%d hdop=%.2f", gga.satellites_tracked, minmea_tofloat(&gga.hdop));
                                            last_no_fix_log = now;
                                        }
                                    }
                                    
                                    else if (gga.fix_quality > 0)
                                    {
                                        if(gps.utc_time == 0) // Register GPS time once
                                            gps.utc_time = gga.time.hours*10000 +
                                            gga.time.minutes*100 +
                                            gga.time.seconds; // HHMMSS format
                                        
                                        gps.latitude  = minmea_tocoord(&gga.latitude);
                                        gps.longitude = minmea_tocoord(&gga.longitude);
                                        gps.altitude = minmea_tofloat(&gga.altitude) - initial_altitude;
                                        
                                        if (initial_altitude == 0.0f) // Measure GPS initial altitude
                                        {
                                            if (altitude_samples < CALIBRATION_SAMPLES)
                                            {
                                                altitude_sum += gps.altitude;
                                                altitude_samples++;
                                                
                                                if (altitude_samples == CALIBRATION_SAMPLES)
                                                {
                                                    initial_altitude = altitude_sum / altitude_samples;
                                                    ESP_LOGI(TAG, "Initial altitude measured: %.2f m", initial_altitude);
                                                }
                                                break; // Set initial altitude before using GPS altitude data
                                            }
                                        }
                                        
                                        // Update global GPS sample
                                        portENTER_CRITICAL(&xGPSMutex);
                                        gps_sample_g.utc_time = gps.utc_time;
                                        gps_sample_g.latitude = gps.latitude;
                                        gps_sample_g.longitude = gps.longitude;
                                        gps_sample_g.altitude = gps.altitude;
                                        gps_sample_g.vel_vertical = gps.vel_vertical;
                                        gps_sample_g.fix = gps.fix;
                                        portEXIT_CRITICAL(&xGPSMutex);
    
                                        xTaskNotify(xTaskAcquire, GPS_BIT, eSetBits); // Notify acquire task that new data is available
                                    }
                                }
                                break;
                            }

                            default:
                                break;
                        }

                        nmea_pos = 0;
                    }
                    else if (nmea_pos < sizeof(nmea_line)-1)
                        nmea_line[nmea_pos++] = c;
                }
            }
        }
    }
}