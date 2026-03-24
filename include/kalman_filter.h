#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Represents a 1D Kalman Filter state.
 * 
 * This structure holds the internal state of the filter.
 * Users should treat this as opaque and use accessor functions.
 */
typedef struct {
    float estimate;          ///< Current state estimate (x)
    float error_covariance;  ///< Estimate error covariance (P)
    float measurement_noise; ///< Measurement noise covariance (R)
    float process_noise;     ///< Process noise covariance (Q)
    float kalman_gain;       ///< Computed Kalman gain (K)
} KalmanFilter;

/**
 * @brief Initializes a Kalman filter instance.
 * 
 * @param kf Pointer to the KalmanFilter structure. Must not be NULL.
 * @param initial_estimate Initial state estimate.
 * @param initial_error Initial error covariance (P0).
 * @param measurement_noise Measurement noise covariance (R).
 * @param process_noise Process noise covariance (Q).
 * @return true if initialization was successful, false otherwise.
 */
bool kalman_init(KalmanFilter* kf, float initial_estimate, 
                 float initial_error, float measurement_noise, 
                 float process_noise);

/**
 * @brief Resets the filter to a new initial state.
 * 
 * @param kf Pointer to the KalmanFilter structure.
 * @param new_estimate New initial state estimate.
 */
void kalman_reset(KalmanFilter* kf, float new_estimate);

/**
 * @brief Performs the prediction step (optional for static systems).
 * 
 * Updates the state estimate and error covariance based on a control input.
 * x_pred = x + u * dt
 * P_pred = P + Q
 * 
 * @param kf Pointer to the KalmanFilter structure.
 * @param control_input Control input (e.g., velocity).
 * @param dt Time delta in seconds.
 */
void kalman_predict(KalmanFilter* kf, float control_input, float dt);

/**
 * @brief Performs the update step with a new measurement.
 * 
 * Implements the standard Kalman update equations:
 * K = P / (P + R)
 * x = x + K * (z - x)
 * P = (1 - K) * P
 * 
 * @param kf Pointer to the KalmanFilter structure.
 * @param measurement New sensor measurement.
 * @return float The updated state estimate.
 */
float kalman_update(KalmanFilter* kf, float measurement);

/**
 * @brief Fuses multiple measurements with varying noise characteristics.
 * 
 * @param kf Pointer to the KalmanFilter structure.
 * @param measurements Array of sensor measurements.
 * @param num_measurements Number of measurements.
 * @param measurement_errors Array of corresponding measurement noise covariances.
 * @return float The final fused estimate.
 */
float kalman_fuse_multiple_sensors(KalmanFilter* kf, const float* measurements, 
                                   size_t num_measurements, const float* measurement_errors);

/**
 * @brief Retrieves the current filter status.
 * 
 * @param kf Pointer to the KalmanFilter structure.
 * @param out_gain Pointer to store the current Kalman gain (can be NULL).
 * @param out_error Pointer to store the current error covariance (can be NULL).
 */
void kalman_get_status(const KalmanFilter* kf, float* out_gain, float* out_error);

/**
 * @brief Validates the filter parameters.
 * 
 * @param kf Pointer to the KalmanFilter structure.
 * @return true if parameters are valid (positive noise values), false otherwise.
 */
bool kalman_validate(const KalmanFilter* kf);

#ifdef __cplusplus
}
#endif

#endif // KALMAN_FILTER_H
