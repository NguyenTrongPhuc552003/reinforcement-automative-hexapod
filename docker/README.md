# Docker Environment for Hexapod Robot

This directory contains Docker configuration for building and running the hexapod robot software in isolated containers. The Docker setup provides a consistent development and deployment environment across different systems.

## Available Containers

### 1. Driver Container (`driver.Dockerfile`)

The driver container provides the low-level hardware interface for the hexapod robot:

- PCA9685 servo driver interface
- IMU sensors (MPU6050, ADXL345)
- GPIO control for ultrasonic sensors
- I2C bus management

### 2. Application Container (`app.Dockerfile`)

The application container runs the high-level control software:

- Kinematics and movement algorithms
- Reinforcement learning models (TD3)
- Gait generation and control
- User interface and command processing

### 3. Python TD3 Container (`pytd3.Dockerfile`)

The Python TD3 container runs the TD3 reinforcement learning algorithm:

- TD3 algorithm implementation
- Training and evaluation scripts
- Integration with the application container

## Building the Containers

### Using Docker Compose (Recommended)

The easiest way to build both containers is using Docker Compose:

```bash
# From the project root directory
docker-compose build
```

### Building Individual Containers

To build containers individually:

```bash
# Build the driver container
./scripts/build.sh -i driver

# Build the application container
./scripts/build.sh -i app

# Build the Python TD3 container
./scripts/build.sh -i pytd3
```

## Running the Containers

### Using Docker Compose (Recommended)

```bash
# Start both containers
docker-compose up

# Run in detached mode
docker-compose up -d

# Stop the containers
docker-compose down
```

### Running Individual Containers

```bash
# Run the driver container with hardware access
docker run --privileged --device=/dev/i2c-1 -v /dev:/dev \
  --name hexapod-driver hexapod-driver

# Run the application container
docker run --network=host --name hexapod-app hexapod-app
```

## Configuration

### Environment Variables

You can configure the containers using environment variables:

- `HEXAPOD_LOG_LEVEL`: Set logging level (DEBUG, INFO, WARNING, ERROR)
- `HEXAPOD_HARDWARE_ENABLED`: Enable/disable hardware interface (true/false)
- `HEXAPOD_SENSOR_TYPE`: Set the sensor type (AUTO, MPU6050, ADXL345)

Example using Docker Compose:

```yaml
services:
  app:
    environment:
      - HEXAPOD_LOG_LEVEL=DEBUG
      - HEXAPOD_SENSOR_TYPE=MPU6050
```

### Volume Mounts

The Docker Compose configuration includes several useful volume mounts:

- `/dev`: Provides access to hardware devices
- `/var/log/hexapod`: Stores log files
- `/data/models`: Persists trained models

## Development Workflow

For development, you can mount your source code as volumes:

```bash
docker run -v $(pwd)/app:/app -v $(pwd)/driver:/driver \
  --privileged --device=/dev/i2c-1 \
  hexapod-driver
```

## Troubleshooting

### Hardware Access Issues

If the container can't access hardware devices:

1. Make sure you're using the `--privileged` flag
2. Verify that device paths are correctly mounted
3. Check that the user in the container has permission to access the devices

### Container Communication Issues

If containers can't communicate:

1. Check that they're on the same network
2. Verify that required ports are exposed
3. Ensure service names resolve correctly in Docker Compose

## Security Considerations

The containers use the `--privileged` flag to access hardware, which grants extensive permissions. For production deployments, consider using more restrictive permissions with specific device mappings.
