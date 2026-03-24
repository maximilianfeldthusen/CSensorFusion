#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "kalman_filter.h"

// Simulated sensor noise generator
static float generate_noisy_measurement(float true_value, float noise_stddev) {
    // Simple approximation of Gaussian noise using Box-Muller transform
    float u1 = (float)rand() / RAND_MAX;
    float u2 = (float)rand() / RAND_MAX;
    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
    return true_value + z0 * noise_stddev;
}

int main(void) {
    srand((unsigned int)time(NULL));

    printf("=== Professional Kalman Filter Demo ===\n\n");

    // 1. Initialization
    KalmanFilter kf;
    if (!kalman_init(&kf, 100.0f, 1.0f, 2.0f, 0.1f)) {
        fprintf(stderr, "Failed to initialize Kalman filter.\n");
        return EXIT_FAILURE;
    }

    printf("Filter initialized successfully.\n");
    printf("Initial Estimate: %.2f m\n\n", kf.estimate);

    // 2. Simulation Setup
    const float true_position = 100.0f;
    const float gps_noise_std = 2.0f;
    const float lidar_noise_std = 1.0f;
    const int iterations = 20;

    printf("Simulating %d sensor readings...\n", iterations);
    printf("%-10s | %-12s | %-12s | %-12s | %-10s\n", 
           "Step", "True Value", "GPS (Noisy)", "Estimate", "Error Cov");
    printf("------------------------------------------------------------\n");

    for (int i = 0; i < iterations; ++i) {
        // Simulate noisy GPS reading
        float gps_reading = generate_noisy_measurement(true_position, gps_noise_std);
        
        // Update filter
        float estimate = kalman_update(&kf, gps_reading);
        
        // Get status for display
        float gain, error;
        kalman_get_status(&kf, &gain, &error);

        printf("%-10d | %-12.2f | %-12.2f | %-12.2f | %-10.3f\n", 
               i + 1, true_position, gps_reading, estimate, error);
    }

    // 3. Multi-Sensor Fusion Example
    printf("\n--- Multi-Sensor Fusion Demo ---\n");
    kalman_reset(&kf, 100.0f); // Reset for fair comparison

    float measurements[] = {100.5f, 99.8f, 101.2f}; // GPS, LiDAR, GPS again
    float noise_levels[] = {2.0f, 1.0f, 2.0f};      // Corresponding noise
    size_t count = sizeof(measurements) / sizeof(measurements[0]);

    float fused_result = kalman_fuse_multiple_sensors(&kf, measurements, count, noise_levels);
    
    printf("Measurements: ");
    for(size_t i=0; i<count; i++) printf("%.2f ", measurements[i]);
    printf("\nFused Estimate: %.2f m\n", fused_result);

    // 4. Validation Check
    if (!kalman_validate(&kf)) {
        printf("Warning: Filter parameters are invalid!\n");
    } else {
        printf("Filter parameters validated.\n");
    }

    printf("\n=== Demo Complete ===\n");
    return EXIT_SUCCESS;
}
