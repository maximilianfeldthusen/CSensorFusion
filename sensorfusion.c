#include <stdio.h>

// Struct to represent sensor data
typedef struct {
    float gps_position;
    float lidar_distance;
    float imu_acceleration;
} SensorData;

// Struct to represent Kalman filter parameters
typedef struct {
    float estimate;
    float error_estimate;
    float measurement_error;
    float gain;
} KalmanFilter;

// Function to update the Kalman filter with new measurement
float kalmanUpdate(KalmanFilter *kf, float measurement) {
    // Compute Kalman gain
    kf->gain = kf->error_estimate / (kf->error_estimate + kf->measurement_error);
    
    // Update estimate
    kf->estimate = kf->estimate + kf->gain * (measurement - kf->estimate);
    
    // Update error estimate
    kf->error_estimate = (1 - kf->gain) * kf->error_estimate;
    
    return kf->estimate;
}

int main() {
    // Initialize sensor data (simulated values)
    SensorData sensor = {100.0, 102.0, 0.5}; // GPS, LiDAR, and IMU values
    
    // Initialize Kalman filter
    KalmanFilter kf = {100.0, 1.0, 2.0, 0.0}; // Initial estimate, error, measurement error
    
    printf("Initial Position Estimate: %.2f\n", kf.estimate);
    
    // Update filter with sensor measurements
    float fused_position = kalmanUpdate(&kf, sensor.gps_position);
    fused_position = kalmanUpdate(&kf, sensor.lidar_distance);
    
    printf("Final Fused Position Estimate: %.2f\n", fused_position);
    
    return 0;
}
