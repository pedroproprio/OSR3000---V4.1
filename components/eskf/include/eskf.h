#ifndef ESKF_H
#define ESKF_H

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KF_N 5

// Variances for Q and R matrices
typedef struct {
    float acc; // acceleromter noise variance (m/s²)²
    float bar; // barometer noise variance (m)²
    float gps_h; // GPS altitude noise variance (m)²
    float gps_vz; // GPS vertical velocity noise variance (m/s)²
    float ba; // accelerometer bias random walk variance (m/s²)²/s
    float bb; // barometer bias random walk variance (m)²/s
    float θe; // angular error random walk variance (rad)²/s
} eskf_var_t;

// Configuration
typedef struct {
    eskf_var_t var;
    float dt; // prediction time step (s)
    float g; // magnitude of acceleration due to gravity (m/s²)
    float igt; // innovation gate threshold for outlier rejection (if r²/S > igt², updates are rejected). e.g., igt = 3 means 3σ gate. igt = 0 means no gating.
    uint16_t idle_samples; // Number of consecutive low-acceleration samples to consider as idle (for ZUPT updates).
} eskf_config_t;

// State vector
typedef struct {
    float h; // altitude (m)
    float vz; // vertical velocity (m/s)
    float ba; // accelerometer bias (m/s²)
    float bb; // barometer bias (m)
    float θe; // angular error (rad)
    float apogee; // apogee (m)
} eskf_state_t;

// Error state vector
typedef struct {
    float h; // altitude error (m)
    float vz; // vertical velocity error (m/s)
    float ba; // accelerometer bias (m/s²)
    float bb; // barometer bias (m)
    float θe; // angular error (rad)
} eskf_error_t;

// Kalman filter instance
typedef struct {
    eskf_config_t cfg;
    eskf_error_t δx;
    eskf_state_t x;

    float P[KF_N][KF_N]; // estimate covariance matrix
    bool initialized; // flag to indicate if the filter has been initialized
} eskf_t;

/**
 * @brief Initialize the Kalman filter Q and P.
 * @param eskf Pointer to the Kalman filter instance.
 */
esp_err_t eskf_init(eskf_t *eskf);

/**
 * @brief Error state and covariance extrapolation.
 * @param eskf Pointer to the Kalman filter instance.
 * @param az Accelerometer z-axis measurement (m/s²).
 * @param xQ Scaling factor (≥ 1) for process noise (e.g., to account for turbulence in boost phase).
 */
void eskf_predict(eskf_t *eskf, float az, float xQ);

/**
 * @brief State, covariance and Kalman gain update with barometer measurement. Resets error state except barometer bias.
 * @param eskf Pointer to the Kalman filter instance.
 * @param h Barometer altitude measurement (m).
 * @param xR Scaling factor (≥ 1) for barometer measurement noise (e.g., to account for dynamic pressure effects).
 * @return true if update was successful, false if rejected by the innovation gate or if filter is not initialized.
 */
bool eskf_update_bar(eskf_t *eskf, float h, float xR);

/**
 * @brief State, covariance and Kalman gain update with GPS measurements. Resets error state except barometer bias.
 * @param eskf Pointer to the Kalman filter instance.
 * @param h GPS altitude measurement (m).
 * @param vz GPS vertical velocity measurement (m/s).
 * @param xR Scaling factor (≥ 1) for GPS measurement noise (e.g., to account for low satellite count).
 * @return true if update was successful, false if rejected by the innovation gate or if filter is not initialized.
 */
bool eskf_update_gps(eskf_t *eskf, float h, float vz, float xR);

#ifdef __cplusplus
}
#endif

#endif // ESKF_H