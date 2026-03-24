

#  1D Kalman Filter (C Implementation)

## Overview

This is a **production-grade 1D Kalman Filter** implementation in C, designed for sensor fusion applications in robotics, IoT, and autonomous systems. The filter estimates the true state of a system by combining noisy sensor measurements with a mathematical model of the system dynamics.

---

## What is a Kalman Filter?

A Kalman Filter is a recursive algorithm that:

1. **Predicts** the next state based on the current state and system model
2. **Updates** the prediction using new sensor measurements
3. **Optimally weights** the prediction vs. measurement based on uncertainty

### Key Concepts

| Term              | Symbol | Meaning                                    |
| ----------------- | ------ | ------------------------------------------ |
| State Estimate    | `x`    | Current best guess of the true value       |
| Error Covariance  | `P`    | Uncertainty in the estimate                |
| Measurement Noise | `R`    | Uncertainty in sensor readings             |
| Process Noise     | `Q`    | Uncertainty in the system model            |
| Kalman Gain       | `K`    | Weight given to measurement vs. prediction |

---

## Code Structure

```text
kalman/
├── include/
│   └── kalman_filter.h      # Public API and data structures
├── src/
│   ├── kalman_filter.c      # Implementation logic
│   └── main.c               # Demo application
├── tests/
│   └── test_kalman.c        # Unit tests
├── Makefile                 # Build automation
└── README.md                # This documentation
```

---

## Core Algorithm

### Mathematical Foundation

The Kalman Filter operates in two phases:

### 1. Prediction Phase

```c
x_pred = x + u * dt;   // State prediction
P_pred = P + Q;        // Covariance prediction
```

### 2. Update Phase

```c
K = P / (P + R);       // Kalman gain
x = x + K * (z - x);   // State update
P = (1 - K) * P;       // Covariance update
```

### Variables

* `x` = state estimate
* `P` = error covariance
* `R` = measurement noise
* `Q` = process noise
* `z` = sensor measurement
* `K` = Kalman gain

---

## API Reference

### Data Structures

```c
typedef struct {
    float estimate;          ///< Current state estimate (x)
    float error_covariance;  ///< Estimate error covariance (P)
    float measurement_noise; ///< Measurement noise covariance (R)
    float process_noise;     ///< Process noise covariance (Q)
    float kalman_gain;       ///< Computed Kalman gain (K)
} KalmanFilter;
```

### Public Functions

| Function                         | Purpose                  | Parameters                                                              |
| -------------------------------- | ------------------------ | ----------------------------------------------------------------------- |
| `kalman_init()`                  | Initialize filter        | `kf, initial_estimate, initial_error, measurement_noise, process_noise` |
| `kalman_reset()`                 | Reset to new state       | `kf, new_estimate`                                                      |
| `kalman_predict()`               | Predict next state       | `kf, control_input, dt`                                                 |
| `kalman_update()`                | Update with measurement  | `kf, measurement`                                                       |
| `kalman_fuse_multiple_sensors()` | Combine multiple sensors | `kf, measurements[], num_measurements, measurement_errors[]`            |
| `kalman_get_status()`            | Query filter state       | `kf, out_gain, out_error`                                               |
| `kalman_validate()`              | Check parameter validity | `kf`                                                                    |

---

## Usage Examples

### Basic Single-Sensor Filtering

```c
#include "kalman_filter.h"

int main(void) {
    KalmanFilter kf;
    
    // Initialize
    kalman_init(&kf, 100.0f, 1.0f, 2.0f, 0.1f);
    
    // Update with sensor reading
    float measurement = 102.5f;
    float estimate = kalman_update(&kf, measurement);
    
    printf("Estimated value: %.2f\n", estimate);
    return 0;
}
```

---

### Multi-Sensor Fusion

```c
float measurements[] = {100.5f, 99.8f, 101.2f};
float noise_levels[] = {2.0f, 1.0f, 2.0f};

float fused = kalman_fuse_multiple_sensors(&kf, measurements, 3, noise_levels);
printf("Fused estimate: %.2f\n", fused);
```

---

### Dynamic System with Motion Model

```c
kalman_predict(&kf, velocity, dt);
float estimate = kalman_update(&kf, measurement);
```

---

## Build Instructions

### Prerequisites

```bash
# Ubuntu/Debian
sudo apt-get install build-essential

# macOS
xcode-select --install
```

### Compilation

```bash
# Standard build
make

# Clean build
make clean && make

# Run demo
make run
```

### Manual Compilation

```bash
gcc -Wall -Wextra -Werror -std=c11 -O2 -I./include \
    src/kalman_filter.c src/main.c -o kalman_demo -lm
```

---

## Parameter Tuning Guide

### Understanding Noise Parameters

| Parameter               | Effect of Increasing          | Typical Values                     |
| ----------------------- | ----------------------------- | ---------------------------------- |
| `measurement_noise (R)` | Filter trusts prediction more | GPS: 2–5, LiDAR: 0.5–2             |
| `process_noise (Q)`     | Filter adapts faster          | Static: 0.01–0.1, Dynamic: 0.1–1.0 |
| `initial_error (P)`     | Faster/slower convergence     | 1.0–10.0                           |

### Tuning Tips

* Start conservative (higher `R`)
* Observe convergence behavior
* Increase `Q` for dynamic systems
* Validate against ground truth

---

## Safety & Validation

### Input Validation

```c
if (kf == NULL) return false;

if (measurement_noise <= 0.0f) {
    fprintf(stderr, "Error: Noise must be positive.\n");
    return false;
}

if (denominator <= 0.0f) {
    kf->kalman_gain = 1.0f;
}

if (kf->error_covariance < 0.0f) {
    kf->error_covariance = 0.0f;
}
```

### Best Practices

 Validate parameters before use
 Check initialization return values
 Use `kalman_validate()`
 Monitor covariance

Avoid negative noise values
 Don’t skip NULL checks

---

## Testing & Verification

### Sample Output

```text
=== Professional Kalman Filter Demo ===

Filter initialized successfully.
Initial Estimate: 100.00 m

Simulating 20 sensor readings...

Step | True Value | GPS (Noisy) | Estimate | Error Cov
------------------------------------------------------
1    | 100.00     | 101.23      | 100.41   | 0.667
2    | 100.00     | 98.76       | 100.09   | 0.444
...
```

### Convergence Indicators

| Metric           | Healthy              | Concerning           |
| ---------------- | -------------------- | -------------------- |
| Error Covariance | Decreases            | Oscillates/Increases |
| Kalman Gain      | Stabilizes (0.1–0.5) | → 0 or 1             |
| Estimate         | Converges            | Drifts               |

---

## Limitations & Extensions

### Current Implementation

| Aspect      | Status              |
| ----------- | ------------------- |
| Dimensions  | 1D only             |
| Linearity   | Linear systems only |
| Noise Model | Gaussian            |
| Platform    | C (POSIX)           |

### Potential Extensions

* Multi-dimensional (2D/3D)
* Extended Kalman Filter (EKF)
* Unscented Kalman Filter (UKF)
* Adaptive noise tuning
* Batch processing

---

## Troubleshooting

| Symptom              | Cause            | Solution   |
| -------------------- | ---------------- | ---------- |
| Divergence           | Q too low        | Increase Q |
| Slow convergence     | P too small      | Increase P |
| Oscillation          | R too low        | Increase R |
| Ignores measurements | R too high       | Decrease R |
| Compile errors       | Missing math lib | Add `-lm`  |

---

## References

* Kalman, R.E. (1960)
* Welch & Bishop (1995)
* Bar-Shalom (2001)

---

## Related Libraries

| Library            | Language | Use Case               |
| ------------------ | -------- | ---------------------- |
| Eigen              | C++      | Matrix operations      |
| filterpy           | Python   | Learning & prototyping |
| robot_localization | ROS      | Production robotics    |

---


## Website 

https://mafeforge.com







