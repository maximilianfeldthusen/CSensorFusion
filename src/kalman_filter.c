#include "kalman_filter.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Internal constants
#define INVALID_PARAM_ERROR -1.0f

bool kalman_init(KalmanFilter* kf, float initial_estimate, 
                 float initial_error, float measurement_noise, 
                 float process_noise) {
    if (kf == NULL) {
        return false;
    }

    // Validate noise parameters (must be positive)
    if (initial_error <= 0.0f || measurement_noise <= 0.0f || process_noise < 0.0f) {
        fprintf(stderr, "Error: Noise parameters must be positive.\n");
        return false;
    }

    kf->estimate = initial_estimate;
    kf->error_covariance = initial_error;
    kf->measurement_noise = measurement_noise;
    kf->process_noise = process_noise;
    kf->kalman_gain = 0.0f;

    return true;
}

void kalman_reset(KalmanFilter* kf, float new_estimate) {
    if (kf == NULL) return;
    
    kf->estimate = new_estimate;
    // Reset error to a reasonable default or keep original? 
    // Usually, we reset P to initial_error, but we don't store it separately.
    // For simplicity, we reset to a standard value or keep current P if not specified.
    // Here we assume the user wants to keep the noise characteristics but reset state.
    kf->kalman_gain = 0.0f;
}

void kalman_predict(KalmanFilter* kf, float control_input, float dt) {
    if (kf == NULL) return;

    // State prediction: x = x + u * dt
    kf->estimate += control_input * dt;

    // Covariance prediction: P = P + Q
    kf->error_covariance += kf->process_noise;
}

float kalman_update(KalmanFilter* kf, float measurement) {
    if (kf == NULL) return 0.0f;

    // Compute Kalman Gain: K = P / (P + R)
    float denominator = kf->error_covariance + kf->measurement_noise;
    
    // Safety check to avoid division by zero
    if (denominator <= 0.0f) {
        // Fallback to measurement if error is degenerate
        kf->kalman_gain = 1.0f;
    } else {
        kf->kalman_gain = kf->error_covariance / denominator;
    }

    // State update: x = x + K * (z - x)
    float innovation = measurement - kf->estimate;
    kf->estimate += kf->kalman_gain * innovation;

    // Covariance update: P = (1 - K) * P
    kf->error_covariance = (1.0f - kf->kalman_gain) * kf->error_covariance;

    // Ensure error covariance doesn't become negative due to floating point errors
    if (kf->error_covariance < 0.0f) {
        kf->error_covariance = 0.0f;
    }

    return kf->estimate;
}

float kalman_fuse_multiple_sensors(KalmanFilter* kf, const float* measurements, 
                                   size_t num_measurements, const float* measurement_errors) {
    if (kf == NULL || measurements == NULL || measurement_errors == NULL || num_measurements == 0) {
        return kf ? kf->estimate : 0.0f;
    }

    float current_estimate = kf->estimate;
    float original_noise = kf->measurement_noise;

    for (size_t i = 0; i < num_measurements; ++i) {
        // Temporarily override measurement noise for this sensor
        kf->measurement_noise = measurement_errors[i];
        
        // Perform update
        current_estimate = kalman_update(kf, measurements[i]);
    }

    // Restore original measurement noise
    kf->measurement_noise = original_noise;

    return current_estimate;
}

void kalman_get_status(const KalmanFilter* kf, float* out_gain, float* out_error) {
    if (kf == NULL) return;

    if (out_gain != NULL) {
        *out_gain = kf->kalman_gain;
    }
    if (out_error != NULL) {
        *out_error = kf->error_covariance;
    }
}

bool kalman_validate(const KalmanFilter* kf) {
    if (kf == NULL) return false;
    return (kf->measurement_noise > 0.0f && kf->process_noise >= 0.0f);
}
