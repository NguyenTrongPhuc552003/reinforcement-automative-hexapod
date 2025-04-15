# Hexapod Quick Start Guide

This guide provides step-by-step instructions to quickly get your hexapod robot up and running.

## Prerequisites

- BeagleBone AI or Black (with Debian/Ubuntu Linux)
- I2C enabled on bus 3
- Hardware components assembled:
  - 18× MG996R servo motors
  - 2× PCA9685 PWM controllers
  - 1× MPU6050 IMU sensor
  - Power supply (6V for servos, 5V logic)
- Host computer with:
  - Docker installed
  - SSH client

## 1. Build the Project

The easiest way to build all components is using our Docker-based build environment.

### Using the Docker Build Environment

From the project root directory:

```bash
# Build all components (kernel modules, user applications, TD3Learn)
./scripts/build.sh

# Or build specific components:
./scripts/build.sh -m    # Build only kernel modules
./scripts/build.sh -u    # Build only user applications
./scripts/build.sh -d    # Build only TD3Learn components
```

The build outputs will be placed in the `deploy` directory.

## 2. Deploy to BeagleBone

After building, deploy to your BeagleBone:

```bash
# Make sure your BeagleBone is reachable first
ping beaglebone.local

# Deploy all components
./scripts/deploy.sh
```

This will copy the built binaries to your BeagleBone and run the installation script.

## 3. Install Kernel Modules

If you haven't used the deploy script or need to manually install:

```bash
# SSH into your BeagleBone
ssh debian@beaglebone.local

# Navigate to the installation directory
cd ~/hexapod_driver

# Install kernel modules
sudo ./install.sh
```

Verify the installation:
```bash
# Check if device node exists
ls -l /dev/hexapod

# Check kernel log for driver messages
dmesg | grep hexapod
```

## 4. Run the Controller Application

To start the interactive controller:

```bash
# With normal output
./hexapod_controller

# With debug output
./hexapod_controller -d
```

### Controller Commands

Once the controller is running, use these keyboard commands:

- **Movement Controls**:
  - `W`/`S`: Move forward/backward
  - `A`/`D`: Rotate left/right
  - `I`/`K`: Raise/lower body
  - `J`/`L`: Tilt left/right

- **Gait Controls**:
  - `1`: Tripod gait
  - `2`: Wave gait
  - `3`: Ripple gait

- **System Controls**:
  - `Space`: Stop and center legs
  - `+`/`-`: Increase/decrease speed
  - `T`: Toggle telemetry display
  - `B`: Toggle balance mode
  - `C`: Center all legs
  - `Q`: Quit application

## 5. Run TD3 Reinforcement Learning

To deploy a trained TD3 model:

```bash
# Deploy a pre-trained model
./td3learn_deploy --model models/trained_model.bin

# Train a new model (requires simulation)
./td3learn_train --config configs/default.yaml
```

## 6. Visualizing Diagrams

To understand the system architecture, review the UML diagrams in `docs/diagrams/out/`:

- **Component Diagram**: Hardware component relationships
- **Deployment Diagram**: System deployment overview
- **Sequence Diagram**: Runtime interaction flows

To rebuild diagrams on your host computer:
```bash
# Build and generate the PNG diagrams from PUML files
./scripts/build.sh -l
```

## 7. Testing

Run individual tests to verify hardware components:

```bash
# Test servo motors
./test_servo

# Test IMU sensor
./test_mpu6050

# Test movement patterns
./test_movement
```

## 8. Calibration

Calibrate your servos for precise positioning:

```bash
# Run calibration utility
./test_calibration

# Apply saved calibration
./test_calibration --apply
```

## Troubleshooting

If you encounter issues:

1. Verify I2C connections with `i2cdetect -y -r 3`
2. Check servo power supply with a multimeter
3. Verify kernel module is loaded with `lsmod | grep hexapod`
4. Check permissions on `/dev/hexapod`
5. Review logs with `dmesg | tail -30`

For detailed troubleshooting, refer to the [Hardware Documentation](hardware.md).
