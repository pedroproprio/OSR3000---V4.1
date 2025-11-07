#include "altitude.h"

static altitude_config_t cfg;
static altitude_state_t x;
static altitude_t alt;

esp_err_t altitude_config(const altitude_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    } else if (config->dt <= 0.0f ||
               config->g == 0.0f ||
               config->accel_var <= 0.0f ||
               config->gps_var_alt <= 0.0f ||
               config->gps_var_vel <= 0.0f ||
               config->bar_var <= 0.0f ||
               config->acc_bias_var <= 0.0f ||
               config->angular_error_var <= 0.0f ||
               (config->use_gps_mahalanobis != 0 &&
               config->use_gps_mahalanobis != 1)) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&cfg, config, sizeof(altitude_config_t));

    alt.F[0][1] = cfg.dt;
    alt.F[0][2] = -0.5f*powf(cfg.dt, 2.0f);
    alt.F[0][3] = -0.5f*powf(cfg.dt, 2.0f)*cfg.g;
    alt.F[1][2] = -cfg.dt;
    alt.F[1][3] = -cfg.dt*cfg.g;

    alt.G[0] = powf(cfg.dt, 2.0f)*0.5f;
    alt.G[1] = cfg.dt;

    alt.Q[0][0] = powf(cfg.dt, 4.0f)*0.25f*cfg.accel_var;
    alt.Q[0][1] = powf(cfg.dt, 3.0f)*0.5f*cfg.accel_var;
    alt.Q[1][0] = alt.Q[0][1];
    alt.Q[1][1] = powf(cfg.dt, 2.0f)*cfg.accel_var;

    alt.P[0][0] = 1.0f/(1.0f/cfg.gps_var_alt + 1.0f/cfg.bar_var);
    alt.P[0][1] = 0.0f;
    alt.P[1][0] = 0.0f;
    alt.P[1][1] = cfg.gps_var_vel;
    return ESP_OK;
}

/*void altitude_predict(const float an) {
    float u = an*cfg.g - x.acc_bias - cfg.g*x.angular_error;
    x.alt = x.alt + alt.F[0][1]*x.vel + alt.F[0][2]*x.acc_bias + alt.F[0][3]*x.angular_error + alt.G[0]*u;
    x.vel = x.vel + alt.F[1][2]*x.acc_bias + alt.F[1][3]*x.angular_error + alt.G[1]*u;

    float FP[4][4];
    FP[0][0] = alt.P[0][0] + alt.F[0][1]*alt.P[1][0] + alt.F[0][2]*alt.P[2][0] + alt.F[0][3]*alt.P[3][0];
    FP[0][1] = alt.P[0][1] + alt.F[0][1]*alt.P[1][1] + alt.F[0][2]*alt.P[2][1] + alt.F[0][3]*alt.P[3][1];
    FP[0][2] = alt.P[0][2] + alt.F[0][1]*alt.P[1][2] + alt.F[0][2]*alt.P[2][2] + alt.F[0][3]*alt.P[3][2];
    FP[0][3] = alt.P[0][3] + alt.F[0][1]*alt.P[1][3] + alt.F[0][2]*alt.P[2][3] + alt.F[0][3]*alt.P[3][3];
    
    FP[1][0] = alt.P[1][0] + alt.F[1][2]*alt.P[2][0] + alt.F[1][3]*alt.P[3][0];
    FP[1][1] = alt.P[1][1] + alt.F[1][2]*alt.P[2][1] + alt.F[1][3]*alt.P[3][1];
    FP[1][2] = alt.P[1][2] + alt.F[1][2]*alt.P[2][2] + alt.F[1][3]*alt.P[3][2];
    FP[1][3] = alt.P[1][3] + alt.F[1][2]*alt.P[2][3] + alt.F[1][3]*alt.P[3][3];

    FP[2][0] = alt.P[2][0] + alt.F[2][3]*alt.P[3][0];
    FP[2][1] = alt.P[2][1] + alt.F[2][3]*alt.P[3][1];
    FP[2][2] = alt.P[2][2] + alt.F[2][3]*alt.P[3][2];
    FP[2][3] = alt.P[2][3] + alt.F[2][3]*alt.P[3][3];

    FP[3][0] = alt.P[3][0];
    FP[3][1] = alt.P[3][1];
    FP[3][2] = alt.P[3][2];
    FP[3][3] = alt.P[3][3];

    alt.P[0][0] = FP[0][0] + FP[0][1]*alt.F[0][1] + FP[0][2]*alt.F[0][2] + FP[0][3]*alt.F[0][3] + alt.Q[0][0];
    alt.P[0][1] = FP[0][1] + FP[0][2]*alt.F[1][2] + FP[0][3]*alt.F[1][3] + alt.Q[0][1];
    alt.P[0][2] = FP[0][2] + FP[0][3]*alt.F[2][3];
    alt.P[0][3] = FP[0][3];

    alt.P[1][0] = FP[1][0] + FP[1][1]*alt.F[0][1] + FP[1][2]*alt.F[0][2] + FP[1][3]*alt.F[0][3] + alt.Q[1][0];
    alt.P[1][1] = FP[1][1] + FP[1][2]*alt.F[1][2] + FP[1][3]*alt.F[1][3] + alt.Q[1][1];
    alt.P[1][2] = FP[1][2] + FP[1][3]*alt.F[2][3];
    alt.P[1][3] = FP[1][3];

    alt.P[2][0] = FP[2][0] + FP[2][1]*alt.F[0][1] + FP[2][2]*alt.F[0][2] + FP[2][3]*alt.F[0][3];
    alt.P[2][1] = FP[2][1] + FP[2][2]*alt.F[1][2] + FP[2][3]*alt.F[1][3];
    alt.P[2][2] = FP[2][2] + FP[2][3]*alt.F[2][3] + cfg.acc_bias_var;
    alt.P[2][3] = FP[2][3];

    alt.P[3][0] = FP[3][0] + FP[3][1]*alt.F[0][1] + FP[3][2]*alt.F[0][2] + FP[3][3]*alt.F[0][3];
    alt.P[3][1] = FP[3][1] + FP[3][2]*alt.F[1][2] + FP[3][3]*alt.F[1][3];
    alt.P[3][2] = FP[3][2] + FP[3][3]*alt.F[2][3];
    alt.P[3][3] = FP[3][3] + cfg.angular_error_var;
}*/

void altitude_update_bar(const float z_meas) {
    float y = z_meas - x.alt;
    float S = alt.P[0][0] + cfg.bar_var;

    // K = P(H^T)(S^-1)
    float K[4];
    K[0] = alt.P[0][0] / S;
    K[1] = alt.P[1][0] / S;
    K[2] = alt.P[2][0] / S;
    K[3] = alt.P[3][0] / S;

    x.alt = x.alt + K[0]*y;
    x.vel = x.vel + K[1]*y;
    x.acc_bias = x.acc_bias + K[2]*y;
    x.angular_error = x.angular_error + K[3]*y;

    // I - KH
    float PI[4];
    PI[0] = (1.0f - K[0]);
    PI[1] = -K[1];
    PI[2] = -K[2];
    PI[3] = -K[3];

    // (PI)P(PI)^T
    float PII[4][4]; 
    PII[0][0] = PI[0]*PI[0]*alt.P[0][0];
    PII[0][1] = PI[0]*PI[1]*alt.P[0][0] + PI[0]*alt.P[0][1];
    PII[0][2] = PI[0]*PI[2]*alt.P[0][0] + PI[0]*alt.P[0][2];
    PII[0][3] = PI[0]*PI[3]*alt.P[0][0] + PI[0]*alt.P[0][3];

    PII[1][0] = PI[0]*(PI[1]*alt.P[0][0] + alt.P[1][0]);
    PII[1][1] = PI[1]*(PI[1]*alt.P[0][0] + alt.P[1][0]) + PI[1]*alt.P[0][1] + alt.P[1][1];
    PII[1][2] = PI[1]*alt.P[0][2] + PI[2]*(PI[1]*alt.P[0][0] + alt.P[1][0]) + alt.P[1][2];
    PII[1][3] = PI[1]*alt.P[0][3] + PI[3]*(PI[1]*alt.P[1][0]) + alt.P[1][3];

    PII[2][0] = PI[0]*(PI[2]*alt.P[0][0] + alt.P[2][0]);
    PII[2][1] = PI[1]*(PI[2]*alt.P[0][0] + alt.P[2][0]) + PI[2]*alt.P[0][1] + alt.P[2][1];
    PII[2][2] = PI[2]*alt.P[0][2] + PI[2]*(PI[2]*alt.P[0][0] + alt.P[2][0]) + alt.P[2][2];
    PII[2][3] = PI[2]*alt.P[0][3] + PI[3]*(PI[2]*alt.P[0][0] + alt.P[2][0]) + alt.P[2][3];

    PII[3][0] = PI[0]*(PI[3]*alt.P[0][0] + alt.P[3][0]);
    PII[3][1] = PI[1]*(PI[3]*alt.P[0][0] + alt.P[3][0]) + PI[3]*alt.P[0][1] + alt.P[3][1];
    PII[3][2] = PI[2]*(PI[3]*alt.P[0][0] + alt.P[3][0]) + PI[3]*alt.P[0][2] + alt.P[3][2];
    PII[3][3] = PI[3]*(PI[3]*alt.P[0][0] + alt.P[3][0]) + PI[3]*alt.P[0][2] + alt.P[3][3];

    // PII + KRK^T
    alt.P[0][0] = PII[0][0] + K[0]*cfg.bar_var*K[0];
    alt.P[0][1] = PII[0][1] + K[0]*cfg.bar_var*K[1];
    alt.P[0][2] = PII[0][2] + K[0]*cfg.bar_var*K[2];
    alt.P[0][3] = PII[0][3] + K[0]*cfg.bar_var*K[3];

    alt.P[1][0] = PII[1][0] + K[1]*cfg.bar_var*K[0];
    alt.P[1][1] = PII[1][1] + K[1]*cfg.bar_var*K[1];
    alt.P[1][2] = PII[1][2] + K[1]*cfg.bar_var*K[2];
    alt.P[1][3] = PII[1][3] + K[1]*cfg.bar_var*K[3];

    alt.P[2][0] = PII[2][0] + K[2]*cfg.bar_var*K[0];
    alt.P[2][1] = PII[2][1] + K[2]*cfg.bar_var*K[1];
    alt.P[2][2] = PII[2][2] + K[2]*cfg.bar_var*K[2];
    alt.P[2][3] = PII[2][3] + K[2]*cfg.bar_var*K[3];

    alt.P[3][0] = PII[3][0] + K[3]*cfg.bar_var*K[0];
    alt.P[3][1] = PII[3][1] + K[3]*cfg.bar_var*K[1];
    alt.P[3][2] = PII[3][2] + K[3]*cfg.bar_var*K[2];
    alt.P[3][3] = PII[3][3] + K[3]*cfg.bar_var*K[3];
}

void altitude_update_gps(const float z_meas_alt, const float z_meas_vel) {
    float y[2];
    y[0] = z_meas_alt - x.alt;
    y[1] = z_meas_vel - x.vel;

    float S[2][2];
    S[0][0] = alt.P[0][0] + cfg.gps_var_alt;
    S[0][1] = alt.P[0][1];
    S[1][0] = alt.P[1][0];
    S[1][1] = alt.P[1][1] + cfg.gps_var_vel;

    float detS = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    if (fabsf(detS) < 1e-12f) {
        ESP_LOGW("Altitude", "GPS Update: S singular");
        return;}
    float Sinv[2][2];
    Sinv[0][0] = S[1][1] / detS;
    Sinv[0][1] = -S[0][1] / detS;
    Sinv[1][0] = -S[1][0] / detS;
    Sinv[1][1] = S[0][0] / detS;

    if(cfg.use_gps_mahalanobis) {
        float maha = y[0]*(Sinv[0][0]*y[0] + Sinv[1][0]*y[1]) + y[1]*(Sinv[0][1]*y[0] + Sinv[1][1]*y[1]);
        const float threshold = 5.99f;
        if (maha > threshold) {
            ESP_LOGW("Altitude", "GPS Update: Mahalanobis test failed (maha=%.2f)", maha);
            return;
        }
    }

    // K = P(H^T)(S^-1)
    float K[4][2];
    K[0][0] = alt.P[0][0]*Sinv[0][0] + alt.P[0][1]*Sinv[1][0];
    K[0][1] = alt.P[0][0]*Sinv[0][1] + alt.P[0][1]*Sinv[1][1];
    K[1][0] = alt.P[1][0]*Sinv[0][0] + alt.P[1][1]*Sinv[1][0];
    K[1][1] = alt.P[1][0]*Sinv[0][1] + alt.P[1][1]*Sinv[1][1];

    K[2][0] = alt.P[2][0]*Sinv[0][0] + alt.P[2][1]*Sinv[1][0];
    K[2][1] = alt.P[2][0]*Sinv[0][1] + alt.P[2][1]*Sinv[1][1];
    K[3][0] = alt.P[3][0]*Sinv[0][0] + alt.P[3][1]*Sinv[1][0];
    K[3][1] = alt.P[3][0]*Sinv[0][1] + alt.P[3][1]*Sinv[1][1];

    x.alt = x.alt + K[0][0]*y[0] + K[0][1]*y[1];
    x.vel = x.vel + K[1][0]*y[0] + K[1][1]*y[1];
    x.acc_bias = x.acc_bias + K[2][0]*y[0] + K[2][1]*y[1];
    x.angular_error = x.angular_error + K[3][0]*y[0] + K[3][1]*y[1];

    // I - KH
    float PI[4][2];
    PI[0][0] = (1.0f - K[0][0]);;
    PI[0][1] = -K[0][1];
    PI[1][0] = -K[1][0];
    PI[1][1] = (1.0f - K[1][1]);
    PI[2][0] = -K[2][0];
    PI[2][1] = -K[2][1];
    PI[3][0] = -K[3][0];
    PI[3][1] = -K[3][1];

    // (PI)P
    float PII[4][4]; 
    PII[0][0] = PI[0][0]*alt.P[0][0] + PI[0][1]*alt.P[1][0];
    PII[0][1] = PI[0][0]*alt.P[0][1] + PI[0][1]*alt.P[1][1];
    PII[0][2] = PI[0][0]*alt.P[0][2] + PI[0][1]*alt.P[1][2];
    PII[0][3] = PI[0][0]*alt.P[0][3] + PI[0][1]*alt.P[1][3];

    PII[1][0] = PI[1][0]*alt.P[0][0] + PI[1][1]*alt.P[1][0];
    PII[1][1] = PI[1][0]*alt.P[0][1] + PI[1][1]*alt.P[1][1];
    PII[1][2] = PI[1][0]*alt.P[0][2] + PI[1][1]*alt.P[1][2];
    PII[1][3] = PI[1][0]*alt.P[0][3] + PI[1][1]*alt.P[1][3];

    PII[2][0] = PI[2][0]*alt.P[0][0] + PI[2][1]*alt.P[1][0] + alt.P[2][0];
    PII[2][1] = PI[2][0]*alt.P[0][1] + PI[2][1]*alt.P[1][1] + alt.P[2][1];
    PII[2][2] = PI[2][0]*alt.P[0][2] + PI[2][1]*alt.P[1][2] + alt.P[2][2];
    PII[2][3] = PI[2][0]*alt.P[0][3] + PI[2][1]*alt.P[1][3] + alt.P[2][3];

    PII[3][0] = PI[3][0]*alt.P[0][0] + PI[3][1]*alt.P[1][0] + alt.P[3][0];
    PII[3][1] = PI[3][0]*alt.P[0][1] + PI[3][1]*alt.P[1][1] + alt.P[3][1];
    PII[3][2] = PI[3][0]*alt.P[0][2] + PI[3][1]*alt.P[1][2] + alt.P[3][2];
    PII[3][3] = PI[3][0]*alt.P[0][3] + PI[3][1]*alt.P[1][3] + alt.P[3][3];

    // PII(PI)^T
    float PIII[4][4];
    PIII[0][0] = PI[0][0]*PII[0][0] + PI[0][1]*PII[0][1];
    PIII[0][1] = PI[1][0]*PII[0][0] + PI[1][1]*PII[0][1];
    PIII[0][2] = PI[2][0]*PII[0][0] + PI[2][1]*PII[0][1] + PII[0][2];
    PIII[0][3] = PI[3][0]*PII[0][0] + PI[3][1]*PII[0][1] + PII[0][3];

    PIII[1][0] = PI[0][0]*PII[1][0] + PI[0][1]*PII[1][1];
    PIII[1][1] = PI[1][0]*PII[1][0] + PI[1][1]*PII[1][1];
    PIII[1][2] = PI[2][0]*PII[1][0] + PI[2][1]*PII[1][1] + PII[1][2];
    PIII[1][3] = PI[3][0]*PII[1][0] + PI[3][1]*PII[1][1] + PII[1][3];

    PIII[2][0] = PI[0][0]*PII[2][0] + PI[0][1]*PII[2][1];
    PIII[2][1] = PI[1][0]*PII[2][0] + PI[1][1]*PII[2][1];
    PIII[2][2] = PI[2][0]*PII[2][0] + PI[2][1]*PII[2][1] + PII[2][2];
    PIII[2][3] = PI[3][0]*PII[2][0] + PI[3][1]*PII[2][1] + PII[2][3];
    
    PIII[3][0] = PI[0][0]*PII[3][0] + PI[0][1]*PII[3][1];
    PIII[3][1] = PI[1][0]*PII[3][0] + PI[1][1]*PII[3][1];
    PIII[3][2] = PI[2][0]*PII[3][0] + PI[2][1]*PII[2][1] + PII[3][2];
    PIII[3][3] = PI[3][0]*PII[3][0] + PI[3][1]*PII[2][1] + PII[3][3];

    // KRK^T
    float PIV[4][4];
    PIV[0][0] = K[0][0]*cfg.gps_var_alt*K[0][0] + K[0][1]*cfg.gps_var_vel*K[0][1];
    PIV[0][1] = K[0][0]*cfg.gps_var_alt*K[1][0] + K[0][1]*cfg.gps_var_vel*K[1][1];
    PIV[0][2] = K[0][0]*cfg.gps_var_alt*K[2][0] + K[0][1]*cfg.gps_var_vel*K[2][1];
    PIV[0][3] = K[0][0]*cfg.gps_var_alt*K[3][0] + K[0][1]*cfg.gps_var_vel*K[3][1];

    PIV[1][0] = K[1][0]*cfg.gps_var_alt*K[0][0] + K[1][1]*cfg.gps_var_vel*K[0][1];
    PIV[1][1] = K[1][0]*cfg.gps_var_alt*K[1][0] + K[1][1]*cfg.gps_var_vel*K[1][1];
    PIV[1][2] = K[1][0]*cfg.gps_var_alt*K[2][0] + K[1][1]*cfg.gps_var_vel*K[2][1];
    PIV[1][3] = K[1][0]*cfg.gps_var_vel*K[3][0] + K[1][1]*cfg.gps_var_vel*K[3][1];

    PIV[2][0] = K[2][0]*cfg.gps_var_alt*K[0][0] + K[2][1]*cfg.gps_var_vel*K[0][1];
    PIV[2][1] = K[2][0]*cfg.gps_var_alt*K[1][0] + K[2][1]*cfg.gps_var_vel*K[1][1];
    PIV[2][2] = K[2][0]*cfg.gps_var_alt*K[2][0] + K[2][1]*cfg.gps_var_vel*K[2][1];
    PIV[2][3] = K[2][0]*cfg.gps_var_vel*K[3][0] + K[2][1]*cfg.gps_var_vel*K[3][1];

    PIV[3][0] = K[3][0]*cfg.gps_var_alt*K[0][0] + K[3][1]*cfg.gps_var_vel*K[0][1];
    PIV[3][1] = K[3][0]*cfg.gps_var_alt*K[1][0] + K[3][1]*cfg.gps_var_vel*K[1][1];
    PIV[3][2] = K[3][0]*cfg.gps_var_alt*K[2][0] + K[3][1]*cfg.gps_var_vel*K[2][1];
    PIV[3][3] = K[3][0]*cfg.gps_var_vel*K[3][0] + K[3][1]*cfg.gps_var_vel*K[3][1];

    // PIII + PIV
    alt.P[0][0] = PIII[0][0] + PIV[0][0];
    alt.P[0][1] = PIII[0][1] + PIV[0][1];
    alt.P[0][2] = PIII[0][2] + PIV[0][2];
    alt.P[0][3] = PIII[0][3] + PIV[0][3];

    alt.P[1][0] = PIII[1][0] + PIV[1][0];
    alt.P[1][1] = PIII[1][1] + PIV[1][1];
    alt.P[1][2] = PIII[1][2] + PIV[1][2];
    alt.P[1][3] = PIII[1][3] + PIV[1][3];

    alt.P[2][0] = PIII[2][0] + PIV[2][0];
    alt.P[2][1] = PIII[2][1] + PIV[2][1];
    alt.P[2][2] = PIII[2][2] + PIV[2][2];
    alt.P[2][3] = PIII[2][3] + PIV[2][3];

    alt.P[3][0] = PIII[3][0] + PIV[3][0];
    alt.P[3][1] = PIII[3][1] + PIV[3][1];
    alt.P[3][2] = PIII[3][2] + PIV[3][2];
    alt.P[3][3] = PIII[3][3] + PIV[3][3];
}

float acc_mahalanobis(const float an) {
    static float window[100];
    static float sum = 0.0f;
    static float sum_sq = 0.0f;
    static uint8_t index = 0;
    static uint8_t count = 0;

    if (count < 100) {
        sum -= window[index];
        sum_sq -= window[index]*window[index];
        window[index] = an;
        sum += an;
        sum_sq += an*an;
        index++;
        if (index >= 100) index = 0;
        count++;
        return an;
    }

    float mean = sum/100;
    float var = (sum_sq/100) - (mean*mean);
    const float alpha = 0.8f; // considering good quality
    float d2 = powf((an-mean), 2.0f)/alpha*cfg.accel_var + (1-alpha)*var;
    if (d2 > 3.84) { // 95% confidence interval
        ESP_LOGW("MAHA", "acc value rejected");
        uint8_t last_valid = (index == 0) ? (100-1) : (index-1);
        return window[last_valid]; 
    }

    sum -= window[index];
    sum_sq -= window[index]*window[index];
    window[index] = an;
    sum += an;
    sum_sq += an*an;
    index++;
    if (index >= 100) index = 0;
    return an;
}

float bar_mahalanobis(const float alt) {
    static float window[100];
    static float sum = 0.0f;
    static float sum_sq = 0.0f;
    static uint8_t index = 0;
    static uint8_t count = 0;

    if (count < 100) {
        sum -= window[index];
        sum_sq -= window[index]*window[index];
        window[index] = alt;
        sum += alt;
        sum_sq += alt*alt;
        index++;
        if (index >= 100) index = 0;
        count++;
        return alt;
    }

    float mean = sum/100;
    float var = (sum_sq/100) - (mean*mean);
    const float alpha = 0.8f; // considering good quality
    float d2 = powf((alt-mean), 2.0f)/alpha*cfg.accel_var + (1-alpha)*var;
    if (d2 > 3.84) { // 95% confidence interval
        ESP_LOGW("MAHA", "acc value rejected");
        uint8_t last_valid = (index == 0) ? (100-1) : (index-1);
        return window[last_valid]; 
    }

    sum -= window[index];
    sum_sq -= window[index]*window[index];
    window[index] = alt;
    sum += alt;
    sum_sq += alt*alt;
    index++;
    if (index >= 100) index = 0;
    return alt;
}

float altitude_get_alt(void) {
    return x.alt;
}

float altitude_get_vel(void) {
    return x.vel;
}

float altitude_get_acc_bias(void) {
    return x.acc_bias;
}

float altitude_get_angular_error(void) {
    return x.angular_error;
}