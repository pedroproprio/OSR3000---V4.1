#ifndef VQF_H
#define VQF_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Struct containing all tuning parameters used by the VQF class.
 *
 * The parameters influence the behavior of the algorithm and are independent of the sampling rate of the IMU data. The
 * constructor sets all parameters to the default values.
 *
 * The parameters #motionBiasEstEnabled, #restBiasEstEnabled, and #magDistRejectionEnabled can be used to enable/disable
 * the main features of the VQF algorithm. The time constants #tauAcc and #tauMag can be tuned to change the trust on
 * the accelerometer and magnetometer measurements, respectively. The remaining parameters influence bias estimation
 * and magnetometer rejection.
 */
typedef struct vqf_params_s {
    /**
     * @brief Time constant \f$\tau_\mathrm{acc}\f$ for accelerometer low-pass filtering in seconds.
     *
     * Small values for \f$\tau_\mathrm{acc}\f$ imply trust on the accelerometer measurements and while large values of
     * \f$\tau_\mathrm{acc}\f$ imply trust on the gyroscope measurements.
     *
     * The time constant \f$\tau_\mathrm{acc}\f$ corresponds to the cutoff frequency \f$f_\mathrm{c}\f$ of the
     * second-order Butterworth low-pass filter as follows: \f$f_\mathrm{c} = \frac{\sqrt{2}}{2\pi\tau_\mathrm{acc}}\f$.
     *
     * Default value: 3.0 s
     */
    float tauAcc;
    /**
     * @brief Time constant \f$\tau_\mathrm{mag}\f$ for magnetometer update in seconds.
     *
     * Small values for \f$\tau_\mathrm{mag}\f$ imply trust on the magnetometer measurements and while large values of
     * \f$\tau_\mathrm{mag}\f$ imply trust on the gyroscope measurements.
     *
     * The time constant \f$\tau_\mathrm{mag}\f$ corresponds to the cutoff frequency \f$f_\mathrm{c}\f$ of the
     * first-order low-pass filter for the heading correction as follows:
     * \f$f_\mathrm{c} = \frac{1}{2\pi\tau_\mathrm{mag}}\f$.
     *
     * Default value: 9.0 s
     */
    float tauMag;

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    /**
     * @brief Enables gyroscope bias estimation during motion phases.
     *
     * If set to true (default), gyroscope bias is estimated based on the inclination correction only, i.e. without
     * using magnetometer measurements.
     */
    int motionBiasEstEnabled;
#endif
    /**
     * @brief Enables rest detection and gyroscope bias estimation during rest phases.
     *
     * If set to true (default), phases in which the IMU is at rest are detected. During rest, the gyroscope bias
     * is estimated from the low-pass filtered gyroscope readings.
     */
    int restBiasEstEnabled;
    /**
     * @brief Enables magnetic disturbance detection and magnetic disturbance rejection.
     *
     * If set to true (default), the magnetic field is analyzed. For short disturbed phases, the magnetometer-based
     * correction is disabled totally. If the magnetic field is always regarded as disturbed or if the duration of
     * the disturbances exceeds #magMaxRejectionTime, magnetometer-based updates are performed, but with an increased
     * time constant.
     */
    int magDistRejectionEnabled;

    /**
     * @brief Standard deviation of the initial bias estimation uncertainty (in degrees per second).
     *
     * Default value: 0.5 °/s
     */
    float biasSigmaInit;
    /**
     * @brief Time in which the bias estimation uncertainty increases from 0 °/s to 0.1 °/s (in seconds).
     *
     * This value determines the system noise assumed by the Kalman filter.
     *
     * Default value: 100.0 s
     */
    float biasForgettingTime;
    /**
     * @brief Maximum expected gyroscope bias (in degrees per second).
     *
     * This value is used to clip the bias estimate and the measurement error in the bias estimation update step. It is
     * further used by the rest detection algorithm in order to not regard measurements with a large but constant
     * angular rate as rest.
     *
     * Default value: 2.0 °/s
     */
    float biasClip;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    /**
     * @brief Standard deviation of the converged bias estimation uncertainty during motion (in degrees per second).
     *
     * This value determines the trust on motion bias estimation updates. A small value leads to fast convergence.
     *
     * Default value: 0.1 °/s
     */
    float biasSigmaMotion;
    /**
     * @brief Forgetting factor for unobservable bias in vertical direction during motion.
     *
     * As magnetometer measurements are deliberately not used during motion bias estimation, gyroscope bias is not
     * observable in vertical direction. This value is the relative weight of an artificial zero measurement that
     * ensures that the bias estimate in the unobservable direction will eventually decay to zero.
     *
     * Default value: 0.0001
     */
    float biasVerticalForgettingFactor;
#endif
    /**
     * @brief Standard deviation of the converged bias estimation uncertainty during rest (in degrees per second).
     *
     * This value determines the trust on rest bias estimation updates. A small value leads to fast convergence.
     *
     * Default value: 0.03 °
     */
    float biasSigmaRest;

    /**
     * @brief Time threshold for rest detection (in seconds).
     *
     * Rest is detected when the measurements have been close to the low-pass filtered reference for the given time.
     *
     * Default value: 1.5 s
     */
    float restMinT;
    /**
     * @brief Time constant for the low-pass filter used in rest detection (in seconds).
     *
     * This time constant characterizes a second-order Butterworth low-pass filter used to obtain the reference for
     * rest detection.
     *
     * Default value: 0.5 s
     */
    float restFilterTau;
    /**
     * @brief Angular velocity threshold for rest detection (in °/s).
     *
     * For rest to be detected, the norm of the deviation between measurement and reference must be below the given
     * threshold. (Furthermore, the absolute value of each component must be below #biasClip).
     *
     * Default value: 2.0 °/s
     */
    float restThGyr;
    /**
     * @brief Acceleration threshold for rest detection (in m/s²).
     *
     * For rest to be detected, the norm of the deviation between measurement and reference must be below the given
     * threshold.
     *
     * Default value: 0.5 m/s²
     */
    float restThAcc;

    /**
     * @brief Time constant for current norm/dip value in magnetic disturbance detection (in seconds).
     *
     * This (very fast) low-pass filter is intended to provide additional robustness when the magnetometer measurements
     * are noisy or not sampled perfectly in sync with the gyroscope measurements. Set to -1 to disable the low-pass
     * filter and directly use the magnetometer measurements.
     *
     * Default value: 0.05 s
     */
    float magCurrentTau;
    /**
     * @brief Time constant for the adjustment of the magnetic field reference (in seconds).
     *
     * This adjustment allows the reference estimate to converge to the observed undisturbed field.
     *
     * Default value: 20.0 s
     */
    float magRefTau;
    /**
     * @brief Relative threshold for the magnetic field strength for magnetic disturbance detection.
     *
     * This value is relative to the reference norm.
     *
     * Default value: 0.1 (10%)
     */
    float magNormTh;
    /**
     * @brief Threshold for the magnetic field dip angle for magnetic disturbance detection (in degrees).
     *
     * Default vaule: 10 °
     */
    float magDipTh;
    /**
     * @brief Duration after which to accept a different homogeneous magnetic field (in seconds).
     *
     * A different magnetic field reference is accepted as the new field when the measurements are within the thresholds
     * #magNormTh and #magDipTh for the given time. Additionally, only phases with sufficient movement, specified by
     * #magNewMinGyr, count.
     *
     * Default value: 20.0
     */
    float magNewTime;
    /**
     * @brief Duration after which to accept a homogeneous magnetic field for the first time (in seconds).
     *
     * This value is used instead of #magNewTime when there is no current estimate in order to allow for the initial
     * magnetic field reference to be obtained faster.
     *
     * Default value: 5.0
     */
    float magNewFirstTime;
    /**
     * @brief Minimum angular velocity needed in order to count time for new magnetic field acceptance (in °/s).
     *
     * Durations for which the angular velocity norm is below this threshold do not count towards reaching #magNewTime.
     *
     * Default value: 20.0 °/s
     */
    float magNewMinGyr;
    /**
     * @brief Minimum duration within thresholds after which to regard the field as undisturbed again (in seconds).
     *
     * Default value: 0.5 s
     */
    float magMinUndisturbedTime;
    /**
     * @brief Maximum duration of full magnetic disturbance rejection (in seconds).
     *
     * For magnetic disturbances up to this duration, heading correction is fully disabled and heading changes are
     * tracked by gyroscope only. After this duration (or for many small disturbed phases without sufficient time in the
     * undisturbed field in between), the heading correction is performed with an increased time constant (see
     * #magRejectionFactor).
     *
     * Default value: 60.0 s
     */
    float magMaxRejectionTime;
    /**
     * @brief Factor by which to slow the heading correction during long disturbed phases.
     *
     * After #magMaxRejectionTime of full magnetic disturbance rejection, heading correction is performed with an
     * increased time constant. This parameter (approximately) specifies the factor of the increase.
     *
     * Furthermore, after spending #magMaxRejectionTime/#magRejectionFactor seconds in an undisturbed magnetic field,
     * the time is reset and full magnetic disturbance rejection will be performed for up to #magMaxRejectionTime again.
     *
     * Default value: 2.0
     */
    float magRejectionFactor;
} vqf_params_t;

typedef struct vqf_handle_t vqf_handle_t;

// Initializes the vqf_params_t struct with default values.
void vqf_params_init(vqf_params_t* params);

// Initializes the object with default parameters.
vqf_handle_t* vqf_init(float gyrTs, float accTs, float magTs);

/**
* @brief Initializes the object with custom parameters.
* @param params VQFParams struct containing the desired parameters
* @param gyrTs sampling time of the gyroscope measurements in seconds
* @param accTs sampling time of the accelerometer measurements in seconds (the value of gyrTs is used if set to -1)
* @param magTs sampling time of the magnetometer measurements in seconds (the value of gyrTs is used if set to -1)
**/
vqf_handle_t* vqf_init_custom(const vqf_params_t* params, float gyrTs, float accTs, float magTs);

/**
* @brief Performs filter update step for one sample (with magnetometer measurement).
* @param handle VQF handle
* @param gyr 3D array containing the gyroscope measurements in rad/s
* @param acc 3D array containing the accelerometer measurements in m/s²
* @param mag 3D array containing the magnetometer measurements in arbitrary units
**/
void vqf_update(vqf_handle_t* handle, const float gyr[3], const float acc[3], const float mag[3]);

/**
* @brief Returns the 9D (with magnetometers) orientation quaternion.
* @param handle VQF handle
* @param out 4D array to store the quaternion (in the order w, x, y, z)
*/
void vqf_get_quat9D(vqf_handle_t* handle, float out[4]);

// Deletes the object and frees the memory.
void vqf_delete(vqf_handle_t* handle);

#ifdef __cplusplus
}
#endif

#endif // VQF_H