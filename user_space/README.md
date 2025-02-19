# Hexapod User Space Applications

This directory contains the user-space applications for controlling and testing the hexapod robot.

## Directory Structure

```
user_space/
├── include/          # Header files
│   ├── hexapod.h    # Main interface definitions
│   └── patterns.h   # Movement pattern definitions
├── src/             # Source files
│   ├── servo_test.c # Servo testing utility
│   └── hexapod.c    # Main control application
└── test/            # Test programs and utilities
```

## Components

### Servo Test Utility
The `servo_test` program provides an interactive menu-driven interface for testing the hexapod:

Features:
1. Test Single Servo
   - Control individual servos by leg and joint
   - Set specific angles for precise positioning
   - Real-time feedback on servo movement

2. Test Leg Movement
   - Control all joints of a leg simultaneously
   - Set hip, knee, and ankle angles
   - Smooth sequential movement

3. Movement Patterns
   - Tripod gait (alternating groups of 3 legs)
   - Wave gait (sequential leg movement)
   - Ripple gait (2-4-6 sequence)
   - Adjustable speed and direction

4. MPU6050 Sensor Reading
   - Read accelerometer data
   - Read gyroscope data
   - Read temperature data
   - Real-time sensor feedback

5. Full System Test
   - Test all legs in sequence
   - Automatic movement through test positions
   - Return to neutral position

### Main Control Application
The main hexapod control application provides:
- High-level movement control
- Sensor data processing
- Movement pattern execution
- Debug interface

## Building

Build all user-space applications:
```bash
make
```

Build specific components:
```bash
make servo_test    # Build servo test utility only
make hexapod      # Build main application only
```

## Usage

### Servo Test
Run the interactive test program:
```bash
sudo ./servo_test
```

The program will present a menu with the following options:
1. Test single servo
   - Enter leg number (0-5)
   - Enter joint number (0-2)
   - Enter angle (-90 to 90)

2. Test leg movement
   - Enter leg number (0-5)
   - Enter hip angle (-90 to 90)
   - Enter knee angle (-90 to 90)
   - Enter ankle angle (-90 to 90)

3. Test movement pattern
   - Enter pattern type (0-2):
     * 0: Tripod gait
     * 1: Wave gait
     * 2: Ripple gait
   - Enter speed (1-100)
   - Enter direction (-1: reverse, 0: stop, 1: forward)

4. Read MPU6050 data
   - Displays current accelerometer readings
   - Displays current gyroscope readings
   - Displays current temperature

5. Test all legs
   - Automatically tests each leg
   - Moves through test positions
   - Returns to neutral position

0. Exit program

Notes:
- Press Ctrl+C at any time to safely stop the program
- The program requires root privileges to access the hardware
- Invalid inputs will be rejected with appropriate error messages

### Main Application
```bash
# Start hexapod control application
sudo ./hexapod

# Start in debug mode
sudo ./hexapod -d
```

## Development

### Adding New Movement Patterns
1. Define pattern in `include/patterns.h`
2. Implement pattern in `src/patterns.c`
3. Add pattern ID to command interface

### Testing
Run the test suite:
```bash
make test
```

### Error Handling
The test utility includes comprehensive error handling:
- Device availability checking
- Input validation
- Safe shutdown on Ctrl+C
- Clear error messages
- Recovery from invalid states
