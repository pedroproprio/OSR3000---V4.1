#include "header.h"

static const char *TAG = "GPS";

void gps_task(data_t *data, uint8_t buffer[GPS_BUFF_SIZE])
{
    int len = uart_read_bytes(UART_NUM_1, buffer, GPS_BUFF_SIZE - 1, pdMS_TO_TICKS(20));
    if (len == -1)
    {
        ESP_LOGE(TAG, "UART read failed");
        return;
    }
    if (len == 0)
    {
        ESP_LOGI(TAG, "UART is empty");
        return;
    }
    buffer[len] = '\0';
    char *line = strtok((char *)buffer, "\r\n");
    while (line != NULL) {
        if (strstr(line, "$GPGGA")) {
            struct minmea_sentence_gga gga;
            if (minmea_parse_gga(&gga, line)) { // call gga parser
                if(utc_time == 1) utc_time = gga.time.hours*1e4+gga.time.minutes*1e2+gga.time.seconds;
                data->latitude = minmea_tocoord(&gga.latitude);
                data->longitude = minmea_tocoord(&gga.longitude);
                data->gps_altitude = gga.altitude.value;
                if (gps_initial_alt == 0) gps_initial_alt = gps_get_initial_alt(gga.altitude.value);
            }
        }
        line = strtok(NULL, "\r\n");
    }
    ubx_parse(buffer, len, data);
    if (STATUS & ARMED)
        altitude_update_gps(data->gps_altitude, data->gps_vel_vertical);
}

bool ubx_parse(uint8_t *buffer, int len, data_t *data)
{
    for (int i=0;i<len-8;i++){
        if (buffer[i] == 0xB5 && buffer[i+2] == 0x62) { // UBX beggining
            uint16_t length = buffer[i+4] | (buffer[i+5] << 8);

            // Copia payload
            uint8_t payload[256];
            memcpy(payload, &buffer[i+6], length);

            float velD = (int32_t)(
                (payload[12]) |
                (payload[13] << 8) |
                (payload[14] << 16) |
                (payload[15] << 24)
            );

            // Read checksum
            uint8_t ck_a = buffer[i+6+length];
            uint8_t ck_b = buffer[i+7+length];

            // Calculates checksum and do integrity check
            uint8_t calc_a, calc_b;
            ubx_calculate_checksum(&buffer[i+2], 4+length, &calc_a, &calc_b);

            if (calc_a == ck_a && calc_b == ck_b) {
                data->gps_vel_vertical = velD;
                return true; // UBX valid
            }
        }
    }
    ESP_LOGW(TAG, "UBX is empty");
    return false ;
}

// Funcao para calcular o checksum UBX
void ubx_calculate_checksum(uint8_t *buffer, uint16_t length, uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = 0;
    *ck_b = 0;
    for (int i=0; i<length; i++) {
        *ck_a += buffer[i];
        *ck_b += *ck_a;
    }
}

float gps_get_initial_alt(const float alt)
{
    static uint8_t i = 0;
    static float sum = 0;
    if (i < 100) { // get mean of 100 samples
        i++;
        sum += alt;
        return 0.0f;
    }
    float mean = sum/100.0f;
    return mean;
}