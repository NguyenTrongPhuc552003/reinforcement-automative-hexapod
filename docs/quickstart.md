# Hexapod Quick Start Guide

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
# Build all components (kernel modules, user applications, pytd3)
./scripts/build.sh

# Or build specific components:
./scripts/build.sh -m    # Build only kernel modules
./scripts/build.sh -u    # Build only user applications
./scripts/build.sh -d    # Build only pytd3 components
```

The build outputs will be placed in the `deploy` directory.

## 2. Train pytd3 Models with Docker

Our Docker-based workflow makes it easy to train and deploy reinforcement learning models:

```bash
# Start the training container
docker-compose run train

# Inside the container, train a model
train my_model

# Or run a specific training script
python3 /app/pytd3/examples/demo.py
```

## 3. Deploy to BeagleBone AI with Hardware Acceleration

After training, you can deploy the model to your BeagleBone AI with hardware acceleration:

```bash
# Using docker-compose
docker-compose run deploy deploy my_model --host beaglebone.local

# Or using the deploy script directly
./pytd3/tools/deploy.sh --model models/my_model --tidl --hardware
```

## 4. Hardware Acceleration Options

The BeagleBone AI offers multiple acceleration options:

- **TIDL (TI Deep Learning)**: Uses DSP and EVE cores for best performance
  ```bash
  # Deploy with TIDL acceleration (4 EVE cores, 2 DSP cores)
  ./pytd3/tools/deploy.sh --model models/my_model --tidl
  ```

- **OpenCL**: Uses GPU for compute operations
  ```bash
  # Deploy with OpenCL acceleration
  ./pytd3/tools/deploy.sh --model models/my_model --opencl
  ```

To check available hardware capabilities:
```bash
./pytd3/tools/hwdetect.sh
```

## 5. Monitoring and Control

Once deployed, you can monitor and control the hexapod:

```bash
# SSH into BeagleBone
ssh debian@beaglebone.local

# Start the pytd3 controller
cd /home/debian/pytd3_model
./run.sh
```

The hexapod will now use the trained reinforcement learning model to control movement.
