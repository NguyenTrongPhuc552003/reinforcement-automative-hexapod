# Hexapod Robot Control Application

## Overview

This application provides a comprehensive control system for a 6-legged (hexapod) robot using advanced Central Pattern Generator (CPG) algorithms. It implements biologically-inspired locomotion control based on Hopf oscillators, providing natural and adaptive movement patterns with autonomous navigation capabilities.

## Features

- **Biological-Inspired Locomotion**: CPG-based control using coupled Hopf oscillators for natural movement patterns
- **Multiple Gait Patterns**: Tripod, Wave, and Ripple gaits with smooth transitions and adaptive parameters
- **Autonomous Navigation**: Real-time obstacle detection and avoidance with ultrasonic sensors
- **Advanced Sensor Integration**: Support for MPU6050 (6-axis) and ADXL345 (3-axis) IMU sensors
- **Adaptive Balance Control**: Real-time balance adjustments using IMU feedback integrated with CPG network
- **Interactive Control**: Real-time keyboard control interface with both manual and autonomous modes
- **Performance Monitoring**: Comprehensive diagnostics and performance metrics
- **Calibration System**: Joint calibration management for mechanical adjustments

## Architecture

The application follows a modern, biologically-inspired architecture with the following key components:

### Central Pattern Generator (CPG) Layer
- **CPG Controller**: High-level coordination of the CPG network for locomotion control
- **CPG Network**: Manages 6 coupled Hopf oscillators (one per leg) for coordinated movement
- **CPG Oscillators**: Individual Hopf oscillators implementing biological neural network dynamics
- **CPG Parameters**: Comprehensive parameter management for oscillator and coupling configuration

### Legacy Motion Control Layer (Backward Compatibility)
- **Legacy Controller**: Traditional control system for comparison and fallback
- **Gait Generator**: Pattern-based movement generation
- **Kinematics Engine**: Mathematical model for leg positioning and inverse kinematics

### Hardware Abstraction Layer
- **Hexapod Interface**: Hardware abstraction for the robot platform
- **Sensor Integration**: IMU and ultrasonic sensor management
- **Calibration System**: Servo calibration and offset management

### Application Layer
- **Application Manager**: Main program orchestration and user interface
- **Autonomous Navigation**: Obstacle detection and path planning
- **Performance Monitor**: Real-time system monitoring and diagnostics

## Building the Application

### Prerequisites

- ARM cross-compiler (arm-linux-gnueabihf-g++)
- C++17 compatible compiler
- Standard development libraries
- Docker environment (recommended)

### Compilation

The project uses Docker-based cross-compilation for consistent builds:

```bash
# Build using Docker (recommended)
./scripts/build.sh -b app

# Debug build with additional information
./scripts/build.sh -b app DEBUG=1

# Run tests for help
./scripts/test.sh -h

# Clean build artifacts
./scripts/build.sh -c app

# Deploy to BeagleBone
./scripts/deploy.sh
```

## Usage

### Basic Controls

| Key                   | Function                               |
|-----------------------|----------------------------------------|
| **Movement**          |                                        |
| W / S                 | Walk forward/backward                  |
| A / D                 | Rotate left/right                      |
| Q / E                 | Strafe left/right                      |
| **Body Control**      |                                        |
| I / K                 | Raise/lower body                       |
| J / L                 | Tilt left/right                        |
| **Gait Control**      |                                        |
| 1 / 2 / 3             | Switch gait (Tripod/Wave/Ripple)       |
| + / -                 | Increase/decrease speed                |
| SPACE                 | Stop and center legs                   |
| **Advanced Features** |                                        |
| B                     | Toggle balance mode                    |
| N                     | Switch to autonomous mode              |
| M                     | Switch to manual mode                  |
| U                     | Toggle ultrasonic obstacle detection   |
| **Monitoring**        |                                        |
| T                     | Toggle telemetry display               |
| P                     | Toggle performance monitoring          |
| [ / ]                 | Adjust balance sensitivity             |
| **System**            |                                        |
| C                     | Run servo diagnostics                  |
| X                     | Switch between MPU6050/ADXL345 sensors |
| H                     | Display help                           |
| R                     | Reset system                           |
| Q                     | Quit application                       |

### CPG Control Features

#### Biological Gait Patterns
The CPG system generates natural movement patterns inspired by biological locomotion:

- **Tripod Gait**: Fast, alternating triangle pattern (legs 0,2,4 vs 1,3,5)
- **Wave Gait**: Slow, sequential wave-like movement for maximum stability
- **Ripple Gait**: Medium-speed ripple pattern for balanced speed and stability

#### Adaptive Control
- Real-time gait adaptation based on terrain feedback
- Automatic balance adjustments using IMU sensor data
- Dynamic frequency and amplitude modulation
- Smooth transitions between gait patterns

#### Autonomous Navigation
- Ultrasonic sensor-based obstacle detection
- Automatic path planning and adjustment
- Emergency stop and avoidance behaviors
- Integration with CPG network for natural responses

### Telemetry Display

When enabled with 'T', the telemetry shows:
- CPG oscillator states and synchronization
- Current gait pattern and parameters
- IMU sensor readings (accelerometer/gyroscope)
- Controller performance metrics
- Autonomous navigation status
- Energy consumption estimates

### Balance Mode

The balance mode uses IMU sensor data integrated with the CPG network to:
- Automatically adjust leg positions for stability
- Compensate for tilting and uneven surfaces
- Maintain natural movement patterns during corrections
- Fine-tune response with '[' and ']' keys

## Test Programs

The `test/` directory contains comprehensive test programs:

- **servo.cpp**: Test servo motor control and calibration
- **mpu6050.cpp**: Test MPU6050 6-axis IMU sensor
- **adxl345.cpp**: Test ADXL345 3-axis accelerometer
- **movement.cpp**: Test CPG-based movement patterns
- **hcsr04.cpp**: Test HC-SR04 ultrasonic distance sensor
- **calibration.cpp**: Test and configure servo calibration
- **balance.cpp**: Test CPG balance integration
- **autonomous.cpp**: Test autonomous navigation features

## CPG Configuration

### Oscillator Parameters
Each leg's oscillator can be configured with:
- Frequency (0.1-5.0 Hz)
- Amplitude (0.0-2.0)
- Phase offset (0-2π radians)
- Coupling strength (0.0-1.0)

### Network Topology
The CPG network supports multiple coupling patterns:
- Nearest neighbor coupling for wave gaits
- Alternating coupling for tripod gaits
- Custom coupling matrices for specialized patterns

### Gait Parameters
Configurable gait parameters include:
- Step frequency and duty cycle
- Stance and swing phase durations
- Step height and length
- Transition smoothness

## Safety Features

- Automatic obstacle detection and avoidance
- Emergency stop functionality with immediate leg centering
- Graceful signal handling for clean shutdown
- Hardware limits enforcement and validation
- Automatic recovery from error states
- Configurable movement speed and acceleration limits

## File Structure

```t
app/
├── src/
│   ├── cpg/                   # CPG implementation
│   │   ├── controller.cpp     # CPG high-level controller
│   │   ├── network.cpp        # CPG oscillator network
│   │   ├── oscillator.cpp     # Individual Hopf oscillators
│   │   └── parameters.cpp     # Parameter management
│   ├── application.cpp        # Main application logic
│   ├── hexapod.cpp            # Hardware abstraction
│   ├── calibration.cpp        # Calibration system
│   ├── ultrasonic.cpp         # Ultrasonic sensor interface
│   ├── common.cpp             # Common utilities
│   └── main.cpp               # Application entry point
├── inc/
│   ├── cpg/                   # CPG headers
│   └── *.hpp                  # Application headers
├── test/                      # Test programs
├── Makefile                   # Build configuration
└── README.md                  # This file
```

## Advanced Features

### CPG Network Visualization
- Real-time oscillator phase display
- Network synchronization monitoring
- Gait pattern visualization
- Performance metrics tracking

### Adaptive Behaviors
- Terrain-based gait selection
- Load-based frequency adjustment
- Obstacle-aware navigation
- Energy-efficient movement optimization

### Development Tools
- Comprehensive debugging output
- Performance profiling tools
- Parameter tuning interfaces
- Real-time system monitoring

## Troubleshooting

### Common Issues

1. **CPG synchronization problems**
   - Check oscillator coupling parameters
   - Verify network initialization
   - Monitor phase relationships

2. **Autonomous navigation issues**
   - Verify ultrasonic sensor connections
   - Check obstacle detection thresholds
   - Ensure CPG integration is enabled

3. **Balance control problems**
   - Calibrate IMU sensor orientation
   - Adjust balance sensitivity parameters
   - Verify CPG balance integration

4. **Performance issues**
   - Monitor CPG update frequency
   - Check system load and timing
   - Optimize parameter configurations

### Debugging Tools

Use the 'P' key to enable performance monitoring, which displays:
- CPG network update rates
- Oscillator synchronization status
- Control loop timing
- Memory usage statistics
- Hardware communication status

## Configuration Files

- `calibration.cfg`: Servo calibration offsets
- `cpg_params.cfg`: CPG oscillator and network parameters
- `navigation.cfg`: Autonomous navigation settings

## Performance Optimization

- CPG networks update at 100Hz for smooth control
- Hardware communication optimized for minimal latency
- Multi-threaded sensor processing
- Adaptive parameter adjustment based on system load

---

For more information about the CPG implementation and biological inspiration, see the technical documentation in the `docs/` directory.
