#!/usr/bin/env python3
"""
Main entry point for Vibration Detection and Rejection from IMU Data project.

This script runs the complete vibration detection and compensation pipeline,
generating plots and results for demonstration purposes.
"""

import numpy as np
import matplotlib.pyplot as plt
from test_vibration_python import (
    vibration_model, vibration_detection, vibration_compensation
)
import os
import sys

def create_output_directory():
    """Create output directory for results if it doesn't exist."""
    output_dir = "vibration_results"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    return output_dir

def run_vibration_detection_demo(output_dir):
    """
    Run complete vibration detection and compensation demonstration.
    
    Args:
        output_dir: Directory to save output plots
    """
    print("=" * 70)
    print("Vibration Detection and Rejection from IMU Data")
    print("=" * 70)
    print()
    
    # Parameters
    fs = 400  # Sampling frequency (Hz)
    duration = 5  # Duration (seconds)
    time = np.arange(0, duration, 1/fs)
    
    print(f"Simulation parameters:")
    print(f"  Sampling frequency: {fs} Hz")
    print(f"  Duration: {duration} seconds")
    print(f"  Number of samples: {len(time)}")
    print()
    
    # Create clean IMU data (static)
    accel_clean = np.zeros((len(time), 3))
    accel_clean[:, 2] = -9.81  # Gravity in Z-axis
    gyro_clean = np.zeros((len(time), 3))
    
    # Test different vibration types
    vibration_types = ['sinusoidal', 'motor', 'rotor', 'random']
    
    for vib_type in vibration_types:
        print(f"\n{'='*70}")
        print(f"Testing {vib_type.upper()} vibration")
        print(f"{'='*70}")
        
        # Configure vibration parameters based on type
        if vib_type == 'sinusoidal':
            vib_params = {
                'frequency': 50,
                'amplitude_accel': [0.5, 0.3, 0.2],
                'amplitude_gyro': [0.1, 0.05, 0.02]
            }
        elif vib_type == 'motor':
            vib_params = {
                'base_freq': 30,
                'harmonics': [1, 2, 3, 4],
                'harmonic_weights': [1.0, 0.6, 0.3, 0.15],
                'amplitude_accel': [1.0, 0.8, 0.6],
                'amplitude_gyro': [0.2, 0.15, 0.1]
            }
        elif vib_type == 'rotor':
            vib_params = {
                'rotor_freq': 35,
                'blade_pass_freq': 70,
                'amplitude_accel': [0.8, 0.6, 1.2],
                'amplitude_gyro': [0.15, 0.12, 0.08],
                'modulation': 0.3
            }
        elif vib_type == 'random':
            vib_params = {
                'freq_band': [20, 100],
                'amplitude_accel': [0.3, 0.25, 0.2],
                'amplitude_gyro': [0.05, 0.04, 0.03]
            }
        
        # Generate vibration
        print(f"\n1. Generating {vib_type} vibration...")
        vibration = vibration_model(time, vib_type, vib_params)
        accel_vibrated = accel_clean + vibration['accel']
        gyro_vibrated = gyro_clean + vibration['gyro']
        
        # Detect vibration
        print(f"2. Detecting vibration...")
        detection_result = vibration_detection(accel_vibrated, gyro_vibrated, fs)
        print(f"   Vibration detected: {detection_result['vibration_detected']}")
        print(f"   Dominant frequency: {detection_result['dominant_freq']:.1f} Hz")
        print(f"   Detection confidence: {detection_result['confidence']:.2f}")
        print(f"   Power ratio: {detection_result['power_ratio']:.4f}")
        
        # Compensate vibration
        print(f"3. Compensating vibration...")
        accel_compensated, gyro_compensated, comp_info = vibration_compensation(
            accel_vibrated, gyro_vibrated, fs, method='notch'
        )
        print(f"   Compensation method: {comp_info['method']}")
        print(f"   Effectiveness: {comp_info['effectiveness']*100:.1f}%")
        
        # Calculate RMS values
        rms_clean = np.sqrt(np.mean(np.sum(accel_clean**2, axis=1)))
        rms_vibrated = np.sqrt(np.mean(np.sum(accel_vibrated**2, axis=1)))
        rms_compensated = np.sqrt(np.mean(np.sum(accel_compensated**2, axis=1)))
        
        print(f"   RMS - Clean: {rms_clean:.4f} m/s²")
        print(f"   RMS - Vibrated: {rms_vibrated:.4f} m/s²")
        print(f"   RMS - Compensated: {rms_compensated:.4f} m/s²")
        
        # Create comprehensive plots
        fig = plt.figure(figsize=(16, 12))
        
        # Plot 1: Time domain - Accelerometer
        ax1 = plt.subplot(3, 3, 1)
        for i, axis in enumerate(['X', 'Y', 'Z']):
            plt.plot(time, accel_vibrated[:, i], label=f'{axis}-axis', alpha=0.7)
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s²)')
        plt.title(f'{vib_type.capitalize()} - Vibrated Accelerometer')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot 2: Time domain - Gyroscope
        ax2 = plt.subplot(3, 3, 2)
        for i, axis in enumerate(['X', 'Y', 'Z']):
            plt.plot(time, gyro_vibrated[:, i], label=f'{axis}-axis', alpha=0.7)
        plt.xlabel('Time (s)')
        plt.ylabel('Angular rate (rad/s)')
        plt.title(f'{vib_type.capitalize()} - Vibrated Gyroscope')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot 3: Frequency domain - Accelerometer
        ax3 = plt.subplot(3, 3, 3)
        accel_mag = np.sqrt(np.sum(accel_vibrated**2, axis=1))
        freqs = np.fft.rfftfreq(len(accel_mag), 1/fs)
        fft_accel = np.abs(np.fft.rfft(accel_mag))
        plt.semilogy(freqs, fft_accel)
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Magnitude')
        plt.title('Accelerometer FFT')
        plt.grid(True, alpha=0.3)
        plt.xlim([0, 150])
        
        # Plot 4: Compensated Accelerometer
        ax4 = plt.subplot(3, 3, 4)
        for i, axis in enumerate(['X', 'Y', 'Z']):
            plt.plot(time, accel_compensated[:, i], label=f'{axis}-axis', alpha=0.7)
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s²)')
        plt.title(f'{vib_type.capitalize()} - Compensated Accelerometer')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot 5: Compensated Gyroscope
        ax5 = plt.subplot(3, 3, 5)
        for i, axis in enumerate(['X', 'Y', 'Z']):
            plt.plot(time, gyro_compensated[:, i], label=f'{axis}-axis', alpha=0.7)
        plt.xlabel('Time (s)')
        plt.ylabel('Angular rate (rad/s)')
        plt.title(f'{vib_type.capitalize()} - Compensated Gyroscope')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot 6: Frequency domain - Compensated
        ax6 = plt.subplot(3, 3, 6)
        accel_comp_mag = np.sqrt(np.sum(accel_compensated**2, axis=1))
        fft_comp = np.abs(np.fft.rfft(accel_comp_mag))
        plt.semilogy(freqs, fft_comp)
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Magnitude')
        plt.title('Compensated Accelerometer FFT')
        plt.grid(True, alpha=0.3)
        plt.xlim([0, 150])
        
        # Plot 7: Vibration signal (isolated)
        ax7 = plt.subplot(3, 3, 7)
        for i, axis in enumerate(['X', 'Y', 'Z']):
            plt.plot(time, vibration['accel'][:, i], label=f'{axis}-axis', alpha=0.7)
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s²)')
        plt.title(f'{vib_type.capitalize()} - Vibration Component')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot 8: Comparison - Z-axis detail
        ax8 = plt.subplot(3, 3, 8)
        samples_per_second = int(fs)  # Calculate samples for 1 second based on sampling frequency
        time_zoom = time[:samples_per_second]  # First 1 second
        plt.plot(time_zoom, accel_clean[:samples_per_second, 2], 'g-', label='Clean', linewidth=2, alpha=0.8)
        plt.plot(time_zoom, accel_vibrated[:samples_per_second, 2], 'r-', label='Vibrated', alpha=0.7)
        plt.plot(time_zoom, accel_compensated[:samples_per_second, 2], 'b--', label='Compensated', linewidth=1.5, alpha=0.7)
        plt.xlabel('Time (s)')
        plt.ylabel('Z-axis Acceleration (m/s²)')
        plt.title('Comparison (First 1 second)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot 9: Detection metrics
        ax9 = plt.subplot(3, 3, 9)
        metrics = ['Detected', 'Confidence', 'Power\nRatio', 'Effectiveness']
        values = [
            1 if detection_result['vibration_detected'] else 0,
            detection_result['confidence'],
            detection_result['power_ratio'],
            comp_info['effectiveness']
        ]
        colors = ['green' if v > 0.5 else 'orange' for v in values]
        plt.bar(metrics, values, color=colors, alpha=0.7)
        plt.ylabel('Value (0-1)')
        plt.title('Detection & Compensation Metrics')
        plt.grid(True, alpha=0.3, axis='y')
        plt.ylim([0, 1.1])
        
        plt.suptitle(f'Vibration Detection and Compensation - {vib_type.capitalize()} Type', 
                     fontsize=14, fontweight='bold')
        plt.tight_layout()
        
        # Save plot
        output_file = os.path.join(output_dir, f'vibration_{vib_type}_results.png')
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"   Plot saved: {output_file}")
        plt.close()
    
    print(f"\n{'='*70}")
    print("All vibration detection tests completed successfully!")
    print(f"Results saved to: {output_dir}/")
    print(f"{'='*70}")
    
def main():
    """Main entry point."""
    try:
        # Create output directory
        output_dir = create_output_directory()
        
        # Run vibration detection demo
        run_vibration_detection_demo(output_dir)
        
        print("\n✓ Vibration Detection and Rejection pipeline completed successfully!")
        print(f"✓ Check '{output_dir}/' directory for generated plots and results")
        
        return 0
        
    except Exception as e:
        print(f"\n✗ Error occurred: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())
