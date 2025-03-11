# Hexapod Quick-Start Guide

This guide helps you get your hexapod robot up and running quickly.

## Prerequisites

- BeagleBone Black with Debian installed
- I2C enabled on bus 2
- 18 servos connected to PCA9685 controllers
- Power supply connected

## Initial Setup

1. **Clone the repository**
   ```bash
   git clone https://github.com/yourusername/reinforcement-automative-hexapod.git
   cd reinforcement-automative-hexapod
   ```

2. **Build all components**
   ```bash
   ./scripts/build.sh
   ```
   This builds both kernel and user-space components and prepares the deployment package.

3. **Install the drivers**
   ```bash
   cd deploy
   sudo ./install.sh
   ```
   
   Verify the driver is loaded:
   ```bash
   lsmod | grep hexapod_driver
   ```

4. **Initialize calibration**
   ```bash
   sudo ./test_calibration reset
   ```
   This creates a default calibration file with all offsets set to zero.

## First Movement Test

1. **Center all servos**
   ```bash
   sudo ./test_servo
   ```
   This will center all servos one by one.

2. **Test individual leg movement**
   ```bash
   sudo ./test_movement leg
   ```
   Each leg will move through its range of motion.

3. **Try walking with tripod gait**
   ```bash
   sudo ./test_movement tripod
   ```
   The robot will walk forward for 15 seconds using the tripod gait pattern.

## Next Steps

- Customize the gait parameters in `app/src/gait.cpp`
- Create a custom control program based on `app/src/main.cpp`
- Fine-tune the servo calibration with `test_calibration apply`
```

## Updating the Calibration Guide

```markdown
// filepath: /home/strongfood/Documents/Hexapod_Programming/reinforcement-automative-hexapod/docs/calibration.md
# Hexapod Calibration Guide

The calibration system ensures precise control of all 18 servos by compensating for mechanical variations in assembly.

## Understanding Calibration

Each servo has a "center" position (0 degrees), but due to mechanical assembly variations, what's truly "center" for your robot may be slightly different. Calibration stores offset values that are automatically applied to servo commands.

For example, if leg 2's hip joint is 5° off, we store an offset of -5° which is automatically applied to all commands to that servo.

## Calibration File

The system uses a binary calibration file (`calibration.cfg`) that contains:
- One record for each leg (6 total)
- Each record contains offsets for hip, knee, and ankle joints

## Calibration Process

### 1. Initial Setup with Default Values

```bash
sudo ./test_calibration reset
```

This creates a default calibration file with all offsets set to zero.

### 2. Visually Check Current Alignment

```bash
sudo ./test_calibration visual
```

This will:
1. Center all legs
2. Move each leg to 15° on all joints
3. Return to center

Watch for legs that don't appear properly aligned.

### 3. Testing Individual Joints

For more precise testing of individual servos:

```bash
sudo ./test_servo 2  # Test leg 2's servos
```

### 4. Creating Custom Calibration

To create a custom calibration with specific values:

```bash
sudo ./test_calibration loadsave
```

This will generate test values and save them to the configuration file. You can modify the values in `test_calibration.cpp` for your specific needs.

### 5. Applying Calibration

Once you've determined the necessary offsets, apply them:

```
sudo ./test_calibration apply
