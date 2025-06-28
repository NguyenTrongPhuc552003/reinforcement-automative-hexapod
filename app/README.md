# Hexapod User-Space Applications

This directory contains the user-space components of the hexapod control system.

## Directory Structure

```
.
├── include/       # Header files
│   ├── gait.h     # Gait control interface
│   ├── hexapod.h  # Hexpod interface
├── src/           # Source files
│   ├── gait.c     # Gait implementation
│   └── main.c     # Main control implementation
└── test/          # Test applications
    ├── test_servo.c
    ├── test_mpu6050.c
    ├── test_calibration.c
    ├── test_movement.c
    └── test_hcsr04.c
```

## Features

### Gait Control
- Tripod gait (alternating groups of 3 legs)
- Wave gait (sequential leg movement)
- Ripple gait (2-4-6 sequence)
- Dynamic gait transitions

### Position Control
- Forward/inverse kinematics
- Joint angle limits
- Smooth movement transitions

### Hardware Interface
- Servo control via kernel driver
- IMU data reading
- Ultrasonic distance sensing
- Error handling and recovery

## Building

```bash
make           # Build all components
make test     # Build test applications
make clean    # Clean build files
```

## Testing

Run individual tests:
```bash
sudo ./test_servo       # Test servo control
sudo ./test_mpu6050     # Test IMU sensor
sudo ./test_movement    # Test movement patterns
sudo ./test_kinematics  # Test kinematics
sudo ./test_hcsr04      # Test ultrasonic sensor
```

## Usage

The application can be run using the main controller:
```bash
sudo ./hexapod_controller
```
    hexapod_cleanup();
    return 0;
}
```

### Gait Control
```c
gait_params_t params = {
    .type = GAIT_TRIPOD,
    .step_height = 30.0,
    .step_length = 50.0,
    .cycle_time = 1.0
};

gait_init(&params);
gait_update(time, &state);
```

## API Documentation

See [API Documentation](../docs/api/user/README.md) for detailed interface descriptions.

## Development

### Adding New Gaits
1. Define gait type in `include/gait.h`
2. Implement gait in `src/gait.c`
3. Add test cases in `test/test_movement.c`

### Testing Guidelines
- Test all new functions
- Verify angle limits
- Check error handling
- Test edge cases
