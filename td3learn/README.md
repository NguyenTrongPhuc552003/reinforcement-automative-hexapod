# TD3Learn - Reinforcement Learning for Hexapod

A module implementing Twin Delayed Deep Deterministic Policy Gradient (TD3) reinforcement learning for training hexapod robot locomotion models.

## Overview

This module provides a complete infrastructure for training and deploying deep reinforcement learning models for hexapod robot locomotion. It's specifically optimized for the BeagleBone AI platform and leverages TI's Deep Learning (TIDL) API for inference acceleration.

## Features

- Complete TD3 algorithm implementation with continuous action space support
- Hardware acceleration via TIDL API and OpenCL
- Modular architecture separating simulation from hardware control
- Seamless integration with the hexapod driver system
- Support for both training and inference workflows
- Real-time adaptation to terrain variations
- Environment abstraction for sim-to-real transfer

## Requirements

- BeagleBone AI with Processor SDK Linux
- TIDL API (included in TI Processor SDK)
- OpenCL runtime (optional)
- C++17 compatible compiler

## Building

### Using the Project's Docker Environment (Recommended)

From the project root directory:
```bash
./scripts/build.sh -d
```

### Manual Build

```bash
cd td3learn
mkdir build && cd build
cmake .. -DENABLE_TIDL=ON -DENABLE_OPENCL=ON
make
```

## Usage

### Training New Models

Train a model using the simulation environment:
```bash
./tools/td3learn_train --config configs/default.yaml --output models/my_model
```

Key training parameters:
- `state_dim`: State dimension (typically 24 for hexapod)
- `action_dim`: Action dimension (typically 18 for hexapod)
- `episodes`: Number of training episodes
- `max_steps`: Maximum steps per episode
- `exploration_noise`: Enable exploration noise

### Deploying Models on Hardware

Deploy a trained model to the hexapod hardware:
```bash
./tools/td3learn_deploy --model models/trained_model.bin --hardware
```

Deploy options:
- `--tidl`: Use TIDL acceleration (requires BeagleBone AI)
- `--opencl`: Use OpenCL acceleration (if available)
- `--simulation`: Test in simulation only
- `--balance`: Enable balance feedback

## Integration with Hexapod System

TD3Learn connects to the main hexapod system through:

1. **Direct hardware interface**: Using the same `/dev/hexapod` device node
2. **Shared kinematics library**: Common coordinate systems and transformations
3. **Unified sensor inputs**: IMU data processing for state representation
4. **Common calibration**: Uses the same calibration data for servo position accuracy

See the [Component Diagram](../docs/diagrams/out/component.png) for visualization of these connections.

## Architecture

The module is structured into several components:

- **Agent**: TD3 algorithm implementation with twin critics and delayed policy updates
- **Environment**: Interface with hexapod hardware or simulation environment
- **Models**: Neural network definitions for actor and critic networks
- **Acceleration**: TIDL and OpenCL integration for hardware acceleration
- **Utils**: Configuration, logging, and visualization tools

Key classes:
- `TD3Agent`: Main reinforcement learning agent
- `HexapodEnvironment`: Interface to the robot hardware
- `Actor`: Policy network for action selection
- `Critic`: Q-value function approximation
- `ReplayBuffer`: Experience replay storage

## File Structure
