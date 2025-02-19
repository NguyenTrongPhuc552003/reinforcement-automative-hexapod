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
The `servo_test` program provides basic testing functionality:
- Test individual servos
- Test leg movements
- Run predefined movement patterns
- Read sensor data

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
```bash
# Test individual servo
./servo_test -s <servo-id> -a <angle>

# Test leg movement
./servo_test -l <leg-id> -c <coxa-angle> -f <femur-angle> -t <tibia-angle>

# Run movement pattern
./servo_test -p <pattern-id>

# Read sensor data
./servo_test -r
```

### Main Application
```bash
# Start hexapod control application
./hexapod

# Start in debug mode
./hexapod -d
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
