#ifndef ALTITUDE_H
#define ALTITUDE_H

#include "string.h"
#include "math.h"
#include "esp_err.h"
#include "esp_log.h"
#include <stdbool.h>

typedef struct {
    float dt;
    float g;
    float accel_var;
    float gps_var_alt;
    float gps_var_vel;
    float bar_var;
    float acc_bias_var;
    float angular_error_var;
    bool use_gps_mahalanobis;
} altitude_config_t;

typedef struct {
    float alt;
    float vel;
    float acc_bias;
    float angular_error;
} altitude_state_t;

typedef struct {
  altitude_config_t config;
  altitude_state_t state;

  float F[3][4];
  float G[2];
  float Q[2][2];
  float P[4][4];
} altitude_t;

esp_err_t altitude_config(const altitude_config_t *config);
void altitude_predict(const float an);
void altitude_update_bar(const float z_meas);
void altitude_update_gps(const float z_meas_alt, const float z_meas_vel);
float acc_mahalanobis(const float an);
float bar_mahalanobis(const float alt);
float altitude_get_alt(void);
float altitude_get_vel(void);
float altitude_get_acc_bias(void);
float altitude_get_angular_error(void);

#endif