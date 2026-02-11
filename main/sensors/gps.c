#include "header.h"

static void ubx_parse_byte(const uint8_t c, gps_sample_t *gps)
{
    static uint8_t state = 0;
    static uint8_t class, id;
    static uint16_t len, cnt;
    static uint8_t ck_a, ck_b;
    static uint8_t payload[64]; // NAV-VELNED tem 36 bytes

    switch(state)
    {
        case 0:
        if(c == 0xB5) state = 1;
        break;
        
        case 1:
        state = (c == 0x62) ? 2 : 0;
        break;
        
        case 2: class=c; ck_a=c; ck_b=c; state=3; break;
        case 3: id=c; ck_a+=c; ck_b+=ck_a; state=4; break;
        case 4: len=c; ck_a+=c; ck_b+=ck_a; state=5; break;
        case 5: len|=(c<<8); ck_a+=c; ck_b+=ck_a; cnt=0; state=6; break;
        
        case 6:
        if(cnt < sizeof(payload))
        payload[cnt] = c;
        ck_a+=c; ck_b+=ck_a;
        cnt++;
        if(cnt>=len) state=7;
        break;
        
        case 7:
        state = (c==ck_a) ? 8 : 0;
        break;
        
        case 8:
        state=0;
        if(c!=ck_b) break;
        
        // NAV-VELNED
        if(class==0x01 && id==0x12 && len>=36)
        {
            int32_t velD = *(int32_t*)&payload[12]; // cm/s (down positive)

            gps->vel_vertical = -(velD / 100.0f); // now m/s (up positive)
        }
        break;
    }
}

static float gps_get_initial_alt(const float alt)
{
    static uint8_t i = 0;
    static float sum = 0;
    
    // get mean of 20 samples
    if (i < 20)
    {
        i++;
        sum += alt;
        return 0.0f;
    }
    float mean = sum/20.0f;
    return mean;
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
    
    ESP_LOGI("GPS", "GPS UART initialized (baud %d)", GPS_BAUDRATE);
}

void gps_task(void *pvParameters)
{
    gps_init();

    char nmea_line[120];
    int nmea_pos = 0;
    
    uint8_t c;
    static float gps_initial_alt = 0;
    gps_sample_t gps;
    
    while (true)
    {
        if (uart_read_bytes(GPS_UART_NUM, &c, 1, pdMS_TO_TICKS(20)) == 1) // 1 byte at a time
        {
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
                    struct minmea_sentence_gga gga;
                    
                    if (minmea_parse_gga(&gga, nmea_line))
                    {
                        if(gps.utc_time == 0) // Register GPS time once
                            gps.utc_time = gga.time.hours*10000 +
                            gga.time.minutes*100 +
                            gga.time.seconds; // HHMMSS format
                        
                        gps.latitude  = minmea_tocoord(&gga.latitude);
                        gps.longitude = minmea_tocoord(&gga.longitude);
                        gps.altitude = minmea_tofloat(&gga.altitude);
                        
                        if (gps_initial_alt == 0)
                            gps_initial_alt = gps_get_initial_alt(minmea_tofloat(&gga.altitude));

                        // Update global GPS sample
                        portENTER_CRITICAL(&xGPSMutex);
                        gps_sample_g.utc_time = gps.utc_time;
                        gps_sample_g.latitude = gps.latitude;
                        gps_sample_g.longitude = gps.longitude;
                        gps_sample_g.altitude = gps.altitude;
                        portEXIT_CRITICAL(&xGPSMutex);

                        xTaskNotifyGive(xTaskAcquire); // Notify acquire task that new data is available
                    }
                    
                    nmea_pos = 0;
                }
                else if (nmea_pos < sizeof(nmea_line)-1)
                    nmea_line[nmea_pos++] = c;
            }
        }
    }
}