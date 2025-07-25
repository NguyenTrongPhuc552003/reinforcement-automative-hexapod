# Hexapod Robot - Quick Start Guide

This guide will help you quickly set up and operate the biologically-inspired hexapod robot with **Central Pattern Generator (CPG)** locomotion control.

**Key Innovation**: This hexapod uses **CPG (Central Pattern Generator) networks** instead of traditional Gait+IK systems, providing natural, adaptive locomotion through biological oscillator patterns.

## Prerequisites

- BeagleBone Black or compatible single-board computer
- Properly assembled hexapod with 18 servos (3 per leg)
- IMU sensor (MPU6050 or ADXL345) for CPG balance feedback
- Optional: Ultrasonic distance sensor for CPG-integrated obstacle avoidance
- Power supply (7.4V-12V recommended) for optimal CPG performance

## Installation

### Option 1: Using Pre-built Image

1. Download the pre-built image from our repository
2. Flash the image to a microSD card
3. Insert the card into your BeagleBone and power on
4. The system will boot automatically

### Option 2: Manual Installation

1. Install dependencies:
   ```bash
   sudo apt update
   sudo apt install -y build-essential cmake git docker
   ```

2. Clone the repository:
   ```bash
   git clone https://github.com/NguyenTrongPhuc552003/reinforcement-automative-hexapod.git
   cd reinforcement-automative-hexapod
   ```

3. Build the project:
   ```bash
   ./scripts/build.sh -b
   ```

4. Deploy the applications to your BeagleBone:
   ```bash
   ./scripts/deploy.sh
   ```

## Hardware Setup

1. Connect the servo controller (PCA9685) to I2C bus
2. Connect the IMU sensor (MPU6050 or ADXL345) to I2C bus
3. Connect ultrasonic sensor (optional) to designated GPIO pins
4. Connect power supply to the servo power rail

## Running the Hexapod

1. Install the hexapod driver:
   ```bash
   ./install.sh
   ```

2. Launch the hexapod application:
   ```bash
   ./hexapod_app
   ```

3. Test the CPG-based movement:
   ```bash
   ./test_autonomous
   ```

## Basic Controls

Once the application is running, use the following keyboard controls for **CPG-based biological locomotion**:

| Key           | Function                                        | CPG Operation                                    |
|---------------|-------------------------------------------------|--------------------------------------------------|
| W             | Walk forward (CPG tripod gait)                  | Oscillator coupling: anti-phase groups           |
| S             | Walk backward (CPG tripod gait)                 | Reverse oscillator direction                     |
| A             | Rotate left (CPG turning pattern)               | Asymmetric oscillator coupling                   |
| D             | Rotate right (CPG turning pattern)              | Asymmetric oscillator coupling                   |
| I / K         | Raise/lower body (CPG amplitude adjustment)     | Modify oscillator amplitude (μ parameter)        |
| J / L         | Tilt left/right (CPG phase modulation)          | Adjust inter-oscillator phase offsets            |
| **1 / 2 / 3** | **Switch CPG gait (Tripod/Wave/Ripple)**        | **Reconfigure coupling matrix dynamically**      |
| **+/-**       | **Increase/decrease CPG frequency**             | **Modify oscillator ω parameter**                |
| Space         | Stop and center legs                            | Reset all oscillators to neutral                 |
| **B**         | **Toggle balance mode (CPG + IMU integration)** | **Enable IMU feedback into oscillator dynamics** |
| \[ / \]       | Adjust balance sensitivity                      | Modify IMU coupling strength                     |
| U             | Toggle ultrasonic sensor                        | Enable CPG obstacle avoidance                    |
| **T**         | **Toggle telemetry display (includes CPG)**     | **Show oscillator phases, coupling strengths**   |
| P             | Toggle performance monitoring                   | Monitor CPG network synchronization              |
| H             | Display help                                    | Show all CPG commands                            |
| Q             | Quit application                                | Graceful CPG network shutdown                    |

## Advanced Features

### CPG-Based Biological Locomotion

The hexapod uses **biologically-inspired Central Pattern Generators** that replace traditional Gait+IK systems:

- **Hopf Oscillators**: Each leg controlled by a nonlinear oscillator with limit cycle dynamics
- **Automatic Coordination**: Phase relationships create stable gaits without manual programming  
- **Natural Adaptation**: Responds to disturbances like biological systems through oscillator coupling
- **Smooth Transitions**: Seamless switching between gait patterns via coupling matrix reconfiguration
- **No Inverse Kinematics**: Direct generation of joint angles from biological patterns

### Mathematical Foundation

The CPG system operates on Hopf oscillator dynamics:
```t
ẋᵢ = (μ - rᵢ²)xᵢ - ωyᵢ + coupling_input
ẏᵢ = (μ - rᵢ²)yᵢ + ωxᵢ
```

Where oscillators naturally synchronize through biological coupling patterns.

### Balance Mode

Enable automatic balancing with the 'B' key. The **CPG system integrates IMU data directly into oscillator dynamics**:
- Use '\[' and '\]' to adjust IMU coupling strength to oscillators
- Balance works with all CPG gait patterns (tripod, wave, ripple)
- **Real-time tilt compensation** through oscillator parameter modulation
- **Biological stability** emerges from limit cycle dynamics

### Sensor Options

The system supports multiple sensor configurations:
- MPU6050: 6-axis IMU with accelerometer and gyroscope
- ADXL345: 3-axis accelerometer
- Switch between them with 'X' and 'C' keys

### Obstacle Avoidance

When ultrasonic sensor is enabled ('U' key):
- Automatically modulates CPG frequency when approaching obstacles
- Integrates obstacle data into oscillator coupling
- Stops and backs away from close obstacles

## Troubleshooting

### Common Issues

1. **Servo jittering**
   - Check power supply voltage
   - Verify calibration values
   - Ensure I2C connection is stable

2. **Unresponsive controls**
   - Check if terminal is in the correct mode
   - Verify driver is loaded (`lsmod | grep hexapod`)
   - Restart the application

3. **IMU errors**
   - Verify I2C connections
   - Try switching sensor type (MPU6050/ADXL345)
   - Run diagnostics with 'P' command

### Running Diagnostics

Use the 'M' key to run comprehensive diagnostics on:
- Servo mapping and calibration
- Controller connectivity
- Sensor readings
- CPG network synchronization
- Oscillator phase relationships

## CPG System Features

The Central Pattern Generator system provides advanced biological locomotion:

### Gait Patterns
- **Tripod**: Fast locomotion with two alternating groups of three legs
- **Wave**: Stable locomotion with sequential leg activation  
- **Ripple**: Medium-speed with ripple-like propagation

### Network Properties
- **Self-Organization**: Patterns emerge automatically from oscillator coupling
- **Robustness**: Natural recovery from disturbances
- **Adaptability**: Responds to terrain changes and obstacles
- **Efficiency**: Biological patterns minimize energy consumption

### Real-time Control
- **50Hz Update Rate**: Smooth, responsive control
- **Phase Coupling**: Automatic leg coordination
- **Parameter Adjustment**: Dynamic gait modification
- **Sensor Integration**: IMU and ultrasonic feedback

## Next Steps

After getting familiar with basic CPG operation:

1. Explore the reinforcement learning module in `pytd3/` for adaptive behavior
2. Customize CPG parameters in the `cpg/` directory in `app/src/`
3. Experiment with oscillator coupling matrices for new gait patterns
4. Create your own sequences and behaviors using the CPG API
5. See the full documentation for advanced CPG programming

## Support

For additional help, please:
- Check the complete documentation in the `docs/` directory
- File issues on our GitHub repository
- Join our community Discord server for real-time assistance
