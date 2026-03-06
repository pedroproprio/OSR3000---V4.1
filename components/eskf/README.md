[![Language](https://img.shields.io/badge/Language-C-navy.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-red?logo=espressif)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
[![Edited with VS Code](https://img.shields.io/badge/Edited_with-VS_Code-007ACC?logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAxMDAgMTAwIj48ZGVmcz48bGluZWFyR3JhZGllbnQgaWQ9ImEiIHgxPSI1MCIgeTE9IjAiIHgyPSI1MCIgeTI9IjEwMCIgZ3JhZGllbnRVbml0cz0idXNlclNwYWNlT25Vc2UiPjxzdG9wIHN0b3AtY29sb3I9IiNmZmYiLz48c3RvcCBvZmZzZXQ9IjEiIHN0b3AtY29sb3I9IiNmZmYiIHN0b3Atb3BhY2l0eT0iMCIvPjwvbGluZWFyR3JhZGllbnQ+PC9kZWZzPjxwYXRoIGZpbGw9IiMwMDY1QTkiIGQ9Ik05Ni40NiAxMC44IDc1Ljg2Ljg4QzczLjQ3LS4yNyA3MC42Mi4yMSA2OC43NSAyLjA4TDEuMyA2My41OGMtMS44MiAxLjY1LTEuODIgNC41MSAwIDYuMTZsNS41MSA1LjAxYzEuNDkgMS4zNSAzLjcyIDEuNDUgNS4zMi4yNGw4MS4yMy02MS42MmMyLjcyLTIuMDcgNi42NC0uMTMgNi42NCAzLjI5di0uMjRjMC0yLjQtMS4zOC00LjU5LTMuNTQtNS42M1oiLz48cGF0aCBmaWxsPSIjMDA3QUNDIiBkPSJNOTYuNDYgODkuMiA3NS44NiA5OS4xMmMtMi4zOSAxLjE1LTUuMjQuNjYtNy4xMS0xLjJMMS4zIDM2LjQyYy0xLjgyLTEuNjUtMS44Mi00LjUxIDAtNi4xN2w1LjUxLTUuMDFjMS40OS0xLjM1IDMuNzItMS40NSA1LjMyLS4yM2w4MS4yMyA2MS42MmMyLjcyIDIuMDcgNi42NC4xMyA2LjY0LTMuM3YuMjRjMCAyLjQtMS4zOCA0LjU5LTMuNTQgNS42M1oiLz48cGF0aCBmaWxsPSIjMUY5Q0YwIiBkPSJNNzUuODYgOTkuMTNjLTIuMzkgMS4xNS01LjI0LjY2LTcuMTEtMS4yMSAyLjMxIDIuMzEgNi4yNS42NiA2LjI1LTIuNlY0LjY3YzAtMy4yNi0zLjk0LTQuODktNi4yNS0yLjU4IDEuODctMS44NyA0LjcyLTIuMzYgNy4xMS0xLjIxbDIwLjYgOS45MWMyLjE2IDEuMDQgMy41NCAzLjIzIDMuNTQgNS42M3Y2Ny4xN2MwIDIuNC0xLjM4IDQuNTktMy41NCA1LjYzbC0yMC42IDkuOTFaIi8+PHBhdGggZmlsbD0idXJsKCNhKSIgZmlsbC1ydWxlPSJldmVub2RkIiBkPSJNNzAuODUgOTkuMzJjMS41OC42MSAzLjM3LjU3IDQuOTYtLjE5bDIwLjU5LTkuOTFjMi4xNi0xLjA0IDMuNTQtMy4yMyAzLjU0LTUuNjNWMTYuNDFjMC0yLjQtMS4zOC00LjU5LTMuNTQtNS42M0w3NS44MS44N2MtMi4wOS0xLTQuNTMtLjc2LTYuMzYuNTgtLjI2LjE5LS41MS40LS43NC42NEwyOS4yOSAzOC4wNGwtMTcuMTctMTMuMDNjLTEuNi0xLjIxLTMuODMtMS4xMS01LjMyLjI0bC01LjUgNS4wMWMtMS44MiAxLjY1LTEuODIgNC41MSAwIDYuMTZMMTYuMTkgNTAgMS4zIDYzLjU4Yy0xLjgyIDEuNjUtMS44MiA0LjUxIDAgNi4xN2w1LjUxIDUuMDFjMS40OSAxLjM1IDMuNzIgMS40NSA1LjMyLjIzbDE3LjE3LTEzLjAzIDM5LjQxIDM1Ljk2Yy42Mi42MiAxLjM2IDEuMDkgMi4xNCAxLjRaTTc0Ljk1IDI3LjMgNDUuMDUgNTBsMjkuOSAyMi43VjI3LjNaIiBvcGFjaXR5PSIuMjUiIHN0eWxlPSJtaXgtYmxlbmQtbW9kZTpvdmVybGF5Ii8+PC9zdmc+&logoColor=white)](https://code.visualstudio.com/)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

# Error State Kalman Filter

## Introduction
This is a simple Error State Kalman Filter (ESKF) for estimating 5 error states:
- Altitude, in m/s²
- Vertical velocity, m/s
- Accelerometer bias, in m/s²
- Barometer bias, in m
- Angular error, in rad

Although estimating the nominal states is not a direct part of the ESKF filter model, they are also provided in the filter to ensure the correct timing on updates and physical coherence.

Altitude `h` and vertical velocity `vz` are estimated by using direct integration from the accelerometer's vertical acceleration data. Small vertical velocity errors can cause significant altitude deviations over time if only altitude error estimation was computed, reinforcing its importance as a state.

The accelerometer bias `ba` is estimated to account for the sensor's slow drift over time and temperature that would cause significant altitude and vertical velocity errors.

The barometer bias `bb` avoids offsets due to environmental conditions, thermal drift or the initial calibration from being interpreted as actual altitude variations.

The angular error `θe` directly impacts the acceleration projection frame on the vertical axis, becoming a very important state. The rocket's trajectory is designed to be slightly inclined relative to the vertical axis. In other words, only small angles are expected. With that, the following simplifications are inferred: $\sin(θ) \approx θ$ and $\cos(θ) \approx 1$. 

`Apogee` is the sixth nominal state, calculated by detecting the transition from positive to non-positive estimated vertical velocity (local maximum) and then registering the highest estimated altitude.

Most implemented Kalman filter models assume a constant acceleration dynamic system due small discrete data acquisition time intervals, among other assumptions. A great guideline for understandig this algorithm can be found [here](https://www.kalmanfilter.net/), an online Kalman Filter tutorial written in 2017 by Alex Becker.

In order to keep the nominal state propagated with nonlinear equations, as it should, the ESKF doesn't try to force a linearization on a highly nonlinear dynamic like that of a rocket. 
Instead, it explicitly separates this state and uses the error state to represent small and slow evolving disturbances around the nominal state. With this approach, the filter always works with small error values and, theoretically, with more robustness to nonlinear flight dynamics and more stability through different flight stages. It can be a more suitable solution to avoid divergence and numerical instability, especially on embedded systems. The error dynamic stays more well-behaved around a linearization, even at critical stages, also allowing a more physical description of the sensors and making the sensor fusion easier. On the other hand, it remains extremely sensitive to tuning.

It is important to emphasize that this filter uses some simplifications in the mathematical realm in order to keep it light and simple. **The information presented in this component has not been validated and should not be considered as such. This is an academic project in development aimed at improving the understanding of the Kalman Filter topic.**

## Additional info
This filter was designed for running on a 3KM rocket as an attempt to estimate with better precision the rocket's altitude and vertical velocity, improving internal logic for the flight, such as deployment coordination.  
The idea is to combine sensors to achieve better results than either of them alone. Since integrating the accelerometer causes significant drift over time, the barometer tends to drift with temperature variation and the GPS loses it's fixes, combining all this data creates a more reliable source of measurements.

The integrations used for the nominal states can be enhanced, for instance, to include aerodynamic drag.

The barometer bias error state, unlike the rest of the error states, is not reset to 0 after an update, it is estimated as a part of the state. That is because it is not used in the model integration, only in the measurement update, so it does not cause instability if not reset.

Joseph's form is used for the estimate covariance update, but still, the P matrix is symmetrized on every step, to avoid diverging due to *float* errors.

Constraints are used on accelerometer bias and angular error nominal states to keep them from diverging from reality and minimize potential errors.

### Hardware

The tests for this library were done in a custom PCB composed of an ESP32 chip (*S3-WROOM-1-N16*), an IMU (*ICM20948*), a barometer (*BMP390*) and a GPS module (*NEO-M8N*).

> This filter uses the **ENU** coordinate system.

## Summary
<div align="center">

| Term  | Name
| :---:  | :---
| δx    | Error State Vector
| x     | Nominal State Vector
| F     | State Transition Matrix
| P     | Estimate Covariance
| Q     | Process Noise Covariance
| R     | Measurement Covariance
| H     | Observation Matrix
| K     | Kalman Gain
| S     | Innovation Covariance
| I     | Identity Matrix
| y     | Innovation
| r     | Residue
<br>
</div>

$$F = \begin{pmatrix}
1 & \Delta t & -0.5\Delta t^2 & 0 & -0.5g\Delta t^2\\
0 & 1 & -\Delta t & 0 & -g\Delta t\\
0 & 0 & 1 & 0 & 0\\
0 & 0 & 0 & 1 & 0\\
0 & 0 & 0 & 0 & 1
\end{pmatrix}$$

<br>

$$Q = \begin{pmatrix}
0.25\sigma _a^2\Delta t^4 & 0.5\sigma _a^2\Delta t^3 & 0 & 0 & 0 \\
0.5\sigma _a^2\Delta t^3 & \sigma _a^2\Delta t^2 & 0 & 0 & 0 \\
0 & 0 & \sigma _{ba}^2\Delta t^2 & 0 & 0 \\
0 & 0 & 0 & \sigma _{bb}^2\Delta t^2 & 0 \\
0 & 0 & 0 & 0 & \sigma _{\theta e}^2\Delta t^2
\end{pmatrix}$$

<br>

$$H_{bar} = \begin{pmatrix} 1 & 0 & 0 & 1 & 0\end{pmatrix}$$

$$R_{bar} = (\sigma _{bar}^2)$$

<br>

$$H_{gps} = \begin{pmatrix}
1 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0
\end{pmatrix}$$

$$R_{gps} = \begin{pmatrix}
\sigma _{h,gps}^2 & 0 \\
0 & \sigma _{vz,gps}^2
\end{pmatrix}$$

## Usage

Include the header:

```c
#include "eskf.h"
```

Configure the filter parameters:

```c
eskf_t eskf;

eskf_var_t var = {
    .acc = 0.25f, // acceleromter noise variance (m/s²)²
    .bar = 1.65f, // barometer noise variance (m)²
    .gps_h = 4.0f, // GPS altitude noise variance (m)²
    .gps_vz = 0.04f, // GPS vertical velocity noise variance (m/s)²
    .ba = 1e-3f, // accelerometer bias random walk variance (m/s²)²/s
    .bb = 1e-3f, // barometer bias random walk variance (m)²/s
    .θe = 1e-6f, // angular error random walk variance (rad)²/s
};

eskf_config_t cfg = {
    .var = var,
    .dt = 0.01f, // prediction time step (s)
    .g = 9.81f, // magnitude of acceleration due to gravity (m/s²)
    .igt = 3.0f, // innovation gate threshold for outlier rejection (if r²/S > igt², updates are rejected). e.g., igt = 3 means 3σ gate. igt = 0 means no gating.
    .idle_samples = 100, // Number of consecutive low-acceleration samples to consider as idle (ZUPT updates)
};

eskf.cfg = cfg;
```
> Note that the accelerometer measurements must be provided in a **constant** frequency. Small frequencies can lead to estimation errors.

> Variances for sensors can be taken from the manucturer datasheet or empirically calculated. The second way usually, if done right, provides more reliable data.  
Variances in the error states depend on the conditions under which the system will be executed.

> The `igt` parameter is the threshold for a simple embedded Mahalanobis distance outiler rejection that runs before the update steps. It can be deactivated if set to 0. It is proportional to the sensor's standart deviation.

> The `idle_sample` is the number of consecutive calls to the prediction step in which the vertical acceleration parameter `az` is low (≤ $3\sigma _{a}$). Once this number is reached, a zero velocity update (ZUPT) is virtually simulated in order to correct drift and allow the filter to remain stable for longer. ZUPTs cease if a non-idle vertical acceleration is detected, resetting the idle sample counter. Additionally, during idle, the corrected vertical acceleration is forced to zero in the prediction step and nominal and error angular states `θe` are reset.

After the configuration step, initialize the filter:

```c
eskf_init(&eskf);
```
<br>

For the prediction step, the vertical acceleration must be provided (with gravity **removed**) through the below function. Although it is not used in the error model, it is needed for residual and integration operations, along with internal logic.  
A multiplier for the Q matrix can also be set (≥ 1), allowing the inflation and deflation of the amount of trust in accelerometer measurements according to the current need.

```c
eskf_predict(eskf_t *eskf, float az, float xQ);
```
<br>

Whenever GPS or barometer data is available, call the bellow functions, respectively, informing the latest measurements (altitude and/or vertical velocity). A multiplier for the R matrix can also be set (≥ 1), allowing the inflation and deflation of the amount of trust in GPS and barometer measurements according to the current need.

```c
bool eskf_update_gps(eskf_t *eskf, float h, float vz, float xR);
bool eskf_update_bar(eskf_t *eskf, float h, float xR);
```
Returns *true* if the update was successful, *false* if rejected by the innovation gate (if enabled) or if the filter is not yet initialized.

> A barometer, obviously, measures pressure, therefore altitude needs to be calculated beforehand.

> Barometer e GPS altitudes **must** have the same reference.

> Setting the multipliers to large values might be more advantageous than zeroing them out.

<br>

The error and nominal states are in the ESKF instance, which can be accessed, respectively, with:

```c
// #include <esp_log.h>
ESP_LOGI("ESKF", "Altitude error: %.2f m", eskf.δx.h);
ESP_LOGI("ESKF", "Vertical velocity error: %.2f m/s", eskf.δx.vz);
ESP_LOGI("ESKF", "Accelerometer bias: %.2f m/s²", eskf.δx.ba);
ESP_LOGI("ESKF", "Barometer bias: %.2f m", eskf.δx.bb); // Redundant
ESP_LOGI("ESKF", "Angular error: %.2f rad", eskf.δx.θe);
```
and
```c
// #include <esp_log.h>
ESP_LOGI("ESKF", "Altitude: %.2f m", eskf.x.h);
ESP_LOGI("ESKF", "Vertical velocity: %.2f m/s", eskf.x.vz);
ESP_LOGI("ESKF", "Accelerometer bias: %.2f m/s²", eskf.x.ba);
ESP_LOGI("ESKF", "Barometer bias: %.2f m", eskf.x.bb);
ESP_LOGI("ESKF", "Angular error: %.2f rad", eskf.x.θe);
ESP_LOGI("ESKF", "Apogee: %.2f m", eskf.x.apogee);
```