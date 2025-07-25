# Hexapod Hardware Documentation

## Overview

This document describes the hardware compo- **Control System**: Via PCA9685 PWM controllers with CPG optimization
  - 2× PCA9685 boards (primary and secondary)
  - I2C communication from BeagleBone Black
  - **50Hz PWM signal generation** for biological rhythm compatibility with CPG networks
  - **Real-time servo command generation** from CPG oscillator outputs

- **CPG Network Integration**:
  - **Real-time Hopf oscillator computation** (50Hz update rate for smooth biological patterns)
  - **Phase-coupled network coordination** between 6 leg oscillators
  - **Biological pattern generation** for natural locomotion without inverse kinematics
  - **Direct servo command mapping** from oscillator outputs to PWM signals
  - **Adaptive coupling matrices** for dynamic gait switching (tripod ↔ wave ↔ ripple)

- **Sensor Feedback for CPG**:
  - **MPU6050 IMU** for orientation detection and **real-time balance feedback to CPG networks**
  - **HC-SR04 Ultrasonic** for obstacle detection with **CPG parameter modulation**
  - **Sensor fusion** integrated directly into oscillator dynamics for adaptive behavior
  - Additional sensor connections available for CPG network expansionhexapod robot with **Central Pattern Generator (CPG)** control system and provides detailed specifications, wiring diagrams, and configuration information. The hardware is specifically designed to support biological pattern generation through CPG networks, eliminating the need for traditional inverse kinematics calculations.

![Hexapod Hardware Overview](../../hardware/README.md)

For detailed visual representation of hardware organization, see the followings:
- [Component Diagram](../diagram/image/component.png)
- [Deployment Diagram](../diagram/image/deployment.png)

## Robot Specifications

**Key Innovation**: This hexapod uses biologically-inspired Central Pattern Generators for locomotion control, replacing traditional Gait+IK systems with natural oscillator networks.

| Parameter                | Value                                               |
|--------------------------|-----------------------------------------------------|
| Dimensions               | 300mm × 300mm × 150mm (L×W×H)                       |
| Weight                   | ~2.0kg (without battery)                            |
| Operating Voltage        | 6V (servos), 5V (logic)                             |
| Power Consumption        | 2-10A depending on CPG-generated movement           |
| Maximum Speed            | ~0.3m/s (CPG-controlled)                            |
| Control Board            | BeagleBone Black                                    |
| Servo Motors             | 18× MG996R (or compatible)                          |
| Degrees of Freedom       | 18 (3 per leg)                                      |
| Operating Time           | ~1 hour (with 2800mAh battery)                      |
| Sensing                  | 6-axis IMU (accelerometer + gyroscope)              |
| **Locomotion System**    | **CPG-based biological pattern generation**         |
| **Control Architecture** | **Central Pattern Generator with Hopf oscillators** |
| **Update Rate**          | **50Hz CPG network computation**                    |
| **Gait Patterns**        | **Tripod, Wave, Ripple (biologically-generated)**   |

## Component List

| Component Description     | Quantity | Notes                            |
|---------------------------|----------|----------------------------------|
| BeagleBone Black          | 1        | Main control board               |
| MG996R Servo Motors       | 18       | 3 per leg (hip, knee, ankle)     |
| PCA9685 PWM Controller    | 2        | 16-channel PWM controller        |
| MPU6050 IMU Sensor        | 1        | 6-axis accelerometer + gyroscope |
| 6V 10A Power Supply       | 1        | For powering servos              |
| Voltage Regulator 5V      | 1        | For logic power                  |
| Aluminum/Plastic Brackets | 18       | For leg joints                   |
| Body Frame                | 1        | Central chassis                  |
| Leg Segments              | 18       | Connecting rods for legs         |
| Foot Pads                 | 6        | Non-slip contact points          |
| Wires and Connectors      | Assorted | For electrical connections       |
| Mounting Hardware         | Assorted | Screws, nuts, standoffs          |

## Mechanical Architecture

The hexapod follows a standard hexapod design with 6 legs arranged radially around a central body:

- **Body**: Hexagonal or circular platform housing the control electronics
- **Legs**: 6 legs with 3 joints each (hip, knee, ankle)
- **Joint Arrangement**:
  - Hip: Rotates horizontally (parallel to body)
  - Knee: Rotates vertically (perpendicular to hip)
  - Ankle: Rotates vertically (perpendicular to knee)

### Dimensions

- **Body Diameter**: 150mm
- **Leg Segments**:
  - Hip to Knee (COXA): 30mm
  - Knee to Ankle (FEMUR): 85mm
  - Ankle to Foot (TIBIA): 130mm
- **Total Leg Extension**: ~245mm fully extended

## Electrical Architecture

### Power System

- **Main Power**: 6V, 10A power supply or battery
- **Power Distribution**:
  - Separate power rails for servos and logic
  - Common ground between all components
- **Protection**:
  - Fuse on main power line (10A)
  - Reverse polarity protection
  - Logic level isolation between BeagleBone and servo power

### Control System

- **Main Controller**: BeagleBone Black
  - AM3358 processor (ARM Cortex-A8)
  - 512MB DDR3 RAM
  - 4GB onboard eMMC
  - Linux operating system with real-time CPG processing

- **Servo Control**: Via PCA9685 PWM controllers
  - 2× PCA9685 boards (primary and secondary)
  - I2C communication from BeagleBone Black
  - 50Hz PWM signal generation for biological rhythm compatibility

- **CPG Integration**:
  - Real-time Hopf oscillator computation (50Hz update rate)
  - Phase-coupled network coordination
  - Biological pattern generation for natural locomotion

- **Sensing**:
  - MPU6050 IMU for orientation detection and balance feedback
  - Additional sensor connections available for expansion

## I2C Bus Configuration

The I2C bus 2 on BeagleBone Black is used for connecting sensors and actuators optimized for CPG network communication:

| Device Description | I2C Address | Bus | Description               | CPG Integration                 |
|------------------- |-------------|-----|---------------------------|---------------------------------|
| MPU6050            | 0x68        | 2   | 6-axis IMU sensor         | Balance feedback to oscillators |
| PCA9685 Primary    | 0x40        | 2   | 16-channel PWM controller | CPG output to servos (legs 0-2) |
| PCA9685 Secondary  | 0x41        | 2   | 16-channel PWM controller | CPG output to servos (legs 3-5) |

### BeagleBone Pin Configuration

| BeagleBone Pin | Function   | Connected To                | CPG Usage                 |
|----------------|------------|-----------------------------|---------------------------| 
| P9_19          | I2C2_SCL   | SCL of all I2C devices      | CPG network communication |
| P9_20          | I2C2_SDA   | SDA of all I2C devices      | CPG network communication |
| P9_1, P9_2     | GND        | Ground for all devices      | Common reference          |
| P9_7, P9_8     | SYS_5V     | Logic power for shields     | CPG controller power      |
| P9_3, P9_4     | DC_3.3V    | 3.3V sensors (e.g. MPU6050) | IMU feedback to CPG       |

## Servo Motor Specifications

Type: MG996R (or compatible) high-torque metal gear servo **optimized for CPG-generated patterns**

| Parameter           | Specification                             | CPG Relevance                    |
|---------------------|-------------------------------------------|----------------------------------|
| Operating Voltage   | 4.8V to 6.6V                              | 6V for fast CPG response         |
| Stall Torque        | 9.4 kg·cm (4.8V) / 11 kg·cm (6.0V)        | Handles CPG-generated loads      |
| Speed               | 0.17 sec/60° (4.8V) / 0.14 sec/60° (6.0V) | Fast enough for 50Hz CPG updates |
| Weight              | 55g                                       | Balanced for biological dynamics |
| Dimensions          | 40.7 × 19.7 × 42.9 mm                     | Compact for hexapod design       |
| Control Signal      | PWM, 1-2ms pulse, **50Hz**                | **Matches CPG update frequency** |
| Rotation Range      | 180° (controlled as ±90°)                 | Full range for CPG trajectories  |
| Working Temperature | -30°C to +60°C                            | Robust operation                 |

**CPG Optimization**: Servos are configured for 50Hz PWM updates to match the CPG network computation frequency, ensuring smooth biological pattern execution.

## Servo Mapping

Each servo is connected to a specific channel on the PCA9685 controllers, with **direct mapping from CPG oscillator outputs**:

| Leg | Joint | PWM Channel | Controller | CPG Oscillator | Coupling Group |
|-----|-------|-------------|------------|----------------|----------------|
| 0   | Hip   | 0           | Primary    | Oscillator 0   | Tripod Group 1 |
|     | Knee  | 1           | Primary    | Oscillator 0   | Tripod Group 1 |
|     | Ankle | 2           | Primary    | Oscillator 0   | Tripod Group 1 |
| 1   | Hip   | 4           | Primary    | Oscillator 1   | Tripod Group 2 |
|     | Knee  | 5           | Primary    | Oscillator 1   | Tripod Group 2 |
|     | Ankle | 6           | Primary    | Oscillator 1   | Tripod Group 2 |
| 2   | Hip   | 8           | Primary    | Oscillator 2   | Tripod Group 1 |
|     | Knee  | 9           | Primary    | Oscillator 2   | Tripod Group 1 |
|     | Ankle | 10          | Primary    | Oscillator 2   | Tripod Group 1 |
| 3   | Hip   | 0           | Secondary  | Oscillator 3   | Tripod Group 2 |
|     | Knee  | 1           | Secondary  | Oscillator 3   | Tripod Group 2 |
|     | Ankle | 2           | Secondary  | Oscillator 3   | Tripod Group 2 |
| 4   | Hip   | 4           | Secondary  | Oscillator 4   | Tripod Group 1 |
|     | Knee  | 5           | Secondary  | Oscillator 4   | Tripod Group 1 |
|     | Ankle | 6           | Secondary  | Oscillator 4   | Tripod Group 1 |
| 5   | Hip   | 8           | Secondary  | Oscillator 5   | Tripod Group 2 |
|     | Knee  | 9           | Secondary  | Oscillator 5   | Tripod Group 2 |
|     | Ankle | 10          | Secondary  | Oscillator 5   | Tripod Group 2 |

**CPG Note**: Each leg is controlled by one CPG oscillator that generates coordinated hip-knee-ankle movements. The coupling groups determine gait patterns (tripod, wave, ripple) through oscillator phase relationships.

## MPU6050 IMU Configuration

The MPU6050 provides **real-time orientation data for CPG balance feedback**:

| Parameter           | Configuration                           | CPG Integration                      |
|---------------------|-----------------------------------------|--------------------------------------|
| I2C Address         | 0x68                                    | Direct to CPG network                |
| Accelerometer Range | ±2g                                     | Tilt detection for balance           |
| Gyroscope Range     | ±500 degrees/second                     | Angular velocity feedback            |
| Sample Rate         | **200Hz**                               | **Fast feedback to CPG oscillators** |
| Low-Pass Filter     | 10Hz                                    | Smooth data for stability            |
| Orientation         | X forward, Y right, Z up                | Standard hexapod reference           |
| **CPG Feedback**    | **Integrated into oscillator dynamics** | **Real-time balance corrections**    |

### Power Management

The MPU6050 has built-in power management features optimized for CPG integration:

**Key features for CPG systems:**
- **Wake-on-read**: Driver automatically detects and wakes device for continuous CPG feedback
- **200Hz sampling**: High-frequency data acquisition for responsive balance control
- **Low-latency integration**: IMU data directly modulates CPG oscillator parameters
- **Clock source**: PLL with X-axis gyroscope reference for stability and precision timing

## CPG System Hardware Requirements

The Central Pattern Generator system has specific hardware requirements for optimal biological pattern generation:

### Real-time Processing
- **Minimum CPU**: ARM Cortex-A8 @ 1GHz (BeagleBone Black meets this requirement)
- **Memory**: 512MB RAM minimum for CPG network computation
- **Update Rate**: 50Hz minimum for smooth oscillator dynamics
- **Latency**: <20ms maximum between sensor input and actuator output

### Servo Response Characteristics
The CPG system requires servos with specific response characteristics:
- **Response Time**: <200ms for 60° movement (MG996R: 140ms @ 6V)
- **Positioning Accuracy**: ±1° for stable biological patterns
- **Holding Torque**: Sufficient to maintain position under CPG-generated loads
- **Speed Consistency**: Uniform speed across the operating range

### Sensor Integration for CPG
- **IMU Sampling Rate**: 100Hz minimum for balance feedback integration
- **Sensor Noise**: Low noise levels for stable oscillator coupling
- **Calibration**: Factory calibration recommended for consistent CPG operation
- **Response Time**: <10ms for real-time CPG parameter adjustment

### Network Synchronization
The CPG system requires precise timing for oscillator synchronization:
- **Clock Stability**: Crystal oscillator recommended for timing accuracy
- **Interrupt Latency**: <1ms for maintaining phase relationships
- **Processing Consistency**: Consistent execution time for CPG update cycles

## Biological Pattern Performance

The hardware configuration supports the following CPG performance characteristics:

| CPG Parameter        | Hardware Requirement | Performance Achieved      |
|----------------------|----------------------|---------------------------|
| Oscillator Frequency | 0.1-10 Hz            | 0.5-5 Hz (walking speeds) |
| Phase Precision      | ±5° accuracy         | ±2° achieved              |
| Coupling Strength    | 0-1.0 range          | Full range supported      |
| Synchronization Time | <3 seconds           | <2 seconds typical        |
| Gait Transition      | <5 seconds           | <3 seconds smooth         |
| Balance Response     | <100ms               | <50ms with IMU            |

This hardware configuration ensures robust, biological-like locomotion patterns with natural coordination and adaptive behavior.
