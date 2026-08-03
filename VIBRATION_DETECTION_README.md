# Vibration Detection and Rejection from IMU Data


## Overview

The project consists of three main components:

1. **Vibration Model** (`vibration_model.m`): Generates realistic vibration signals that can be added to clean IMU data for testing and validation.

2. **Vibration Detection** (`vibration_detection.m`): Detects the presence of vibration in IMU data using multiple analysis methods.

3. **Vibration Compensation** (`vibration_compensation.m`): Removes or reduces vibration artifacts from IMU measurements using various filtering techniques.

## Features

### Vibration Types Supported
- **Sinusoidal**: Pure sinusoidal vibrations at specified frequencies
- **Motor**: Multi-harmonic vibrations typical of electric motors
- **Rotor**: Quadcopter/UAV rotor-induced vibrations with modulation
- **Random**: Band-limited random vibration

### Detection Methods
- **Frequency Domain**: Power spectral density analysis
- **Variance-based**: Statistical variance analysis in sliding windows
- **Energy-based**: Energy detection in specified frequency bands
- **Combined**: Fusion of multiple detection methods

### Compensation Methods
- **Low-pass Filtering**: Simple frequency domain filtering
- **Notch Filtering**: Targeted removal of specific frequencies
- **Adaptive Filtering**: LMS-based adaptive vibration cancellation
- **Spectral Subtraction**: Frequency domain vibration suppression

## Files Structure

```
├── vibration_model.m           # Vibration signal generation
├── vibration_detection.m       # Vibration detection algorithms
├── vibration_compensation.m    # Vibration removal methods
├── vibration_demo.m           # Comprehensive demo script
├── vibration_example_simple.m  # Simple usage example
└── run_vibration_tests.m      # Test suite

test_vibration_python.py       # Python implementation and tests
```

## Quick Start


```matlab
% Parameters
fs = 400;                    % Sampling frequency
time = (0:1/fs:5-1/fs)';    % 5 seconds of data

% Create clean IMU data
accel_clean = [zeros(length(time), 2), -9.81*ones(length(time), 1)];
gyro_clean = zeros(length(time), 3);

% Add vibration
vib_params.frequency = 50;
vib_params.amplitude_accel = [0.5, 0.3, 0.2];
vib_params.amplitude_gyro = [0.1, 0.05, 0.02];

vibration = vibration_model(time, 'sinusoidal', vib_params);
accel_vibrated = accel_clean + vibration.accel;
gyro_vibrated = gyro_clean + vibration.gyro;

% Detect vibration
detection_result = vibration_detection(accel_vibrated, gyro_vibrated, fs);
fprintf('Vibration detected: %s\n', string(detection_result.vibration_detected));
fprintf('Dominant frequency: %.1f Hz\n', detection_result.dominant_freq);

% Compensate vibration
[accel_compensated, gyro_compensated, comp_info] = ...
    vibration_compensation(accel_vibrated, gyro_vibrated, fs);
fprintf('Compensation effectiveness: %.1f%%\n', comp_info.effectiveness * 100);
```

### Running the Demo

```matlab
% Comprehensive demo with all vibration types and methods
vibration_demo

% Simple example
vibration_example_simple

% Test suite
run_vibration_tests
```

## API Reference

### vibration_model(time, vib_type, vib_params)

Generates vibration signals for IMU simulation.

**Parameters:**
- `time`: Time vector (Nx1) in seconds
- `vib_type`: String - 'sinusoidal', 'motor', 'rotor', 'random'
- `vib_params`: Structure with vibration parameters

**Returns:**
- Structure with fields: `accel` (Nx3), `gyro` (Nx3), `freq`, `type`

**Example Parameters:**
```matlab
% Sinusoidal vibration
params.frequency = 50;                    % Hz
params.amplitude_accel = [0.5, 0.3, 0.2]; % m/s^2
params.amplitude_gyro = [0.1, 0.05, 0.02]; % rad/s

% Motor vibration  
params.base_freq = 30;                    % Hz
params.harmonics = [1, 2, 3, 4];
params.harmonic_weights = [1.0, 0.6, 0.3, 0.15];
params.amplitude_accel = [1.0, 0.8, 0.6];
params.amplitude_gyro = [0.2, 0.15, 0.1];
```

### vibration_detection(accel, gyro, fs, ...)

Detects presence of vibration in IMU data.

**Parameters:**
- `accel`: Accelerometer data (Nx3) in m/s²
- `gyro`: Gyroscope data (Nx3) in rad/s  
- `fs`: Sampling frequency in Hz
- Optional: `'Method'`, `'FreqRange'`, `'ThresholdFactor'`, etc.

**Returns:**
- Structure with detection results including `vibration_detected`, `dominant_freq`, `confidence`

### vibration_compensation(accel, gyro, fs, ...)

Removes vibration signals from IMU data.

**Parameters:**
- `accel`: Accelerometer data (Nx3) in m/s²
- `gyro`: Gyroscope data (Nx3) in rad/s
- `fs`: Sampling frequency in Hz
- Optional: `'Method'`, `'AutoDetect'`, compensation parameters

**Returns:**
- `[accel_clean, gyro_clean, compensation_info]`

## Applications

### UAV/Drone Applications
The vibration processing is particularly useful for:
- Quadcopter flight control systems
- Gimbal stabilization
- Navigation system improvements
- Motor vibration isolation

### Example: Quadcopter Vibration Processing
```matlab
% Typical quadcopter parameters
fs = 400;
rotor_rpm = 2100;           % RPM
rotor_freq = rotor_rpm / 60; % Convert to Hz

% Configure rotor vibration
params.rotor_freq = rotor_freq;
params.blade_pass_freq = rotor_freq * 2; % 2-blade rotor
params.amplitude_accel = [0.8, 0.6, 1.2]; % Stronger in Z-axis
params.modulation = 0.3;    % Amplitude modulation

% Add to IMU data and process
vibration = vibration_model(time, 'rotor', params);
% ... process as shown above
```

## Performance Metrics

The algorithms provide several effectiveness metrics:

- **Detection Confidence**: 0-1 scale indicating detection certainty
- **Power Ratio**: Ratio of vibration band power to total signal power
- **Compensation Effectiveness**: Ratio of vibration power reduction
- **RMS Error Improvement**: Percentage reduction in RMS error vs clean data

## Validation and Testing

The implementation includes comprehensive test suites:

```matlab
run_vibration_tests  % Comprehensive test suite with validation
```

### Python Tests  
```python
.venv/bin/python test_vibration_python.py  # Python validation and benchmarking
```

## Algorithm Details

### Detection Algorithm
1. **Multi-domain Analysis**: Combines frequency, time, and energy domain features
2. **Adaptive Thresholds**: Automatically adjusts detection thresholds based on signal characteristics  
3. **Minimum Duration Filtering**: Removes spurious short-duration detections
4. **Confidence Scoring**: Provides quantitative confidence measure

### Compensation Algorithm
1. **Auto-detection**: Automatically identifies vibration characteristics
2. **Method Selection**: Multiple compensation approaches for different scenarios
3. **Effectiveness Measurement**: Quantifies compensation performance
4. **Preservation of Motion**: Maintains low-frequency motion information

## Technical Specifications

- **Sampling Rates**: Optimized for 100-1000 Hz IMU data
- **Frequency Range**: Effective for 10-500 Hz vibrations  
- **Latency**: Real-time capable with appropriate buffering
- **Memory Usage**: Minimal memory footprint for embedded applications

## Future Enhancements

Potential improvements and extensions:

1. **Machine Learning**: Deep learning-based vibration classification
2. **Multi-sensor Fusion**: Integration with magnetometer and other sensors
3. **Real-time Implementation**: Optimized for real-time embedded processing
4. **Calibration Tools**: Automated parameter tuning for specific applications

## References

This implementation is based on established vibration analysis techniques and IMU processing methods commonly used in:

- Aerospace navigation systems
- Robotics and autonomous vehicles  
- MEMS sensor signal processing
- Digital signal processing for mechanical systems

The algorithms are designed to be robust, computationally efficient, and suitable for both offline analysis and real-time applications.