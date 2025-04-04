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

## Requirements

- BeagleBone AI with Processor SDK Linux
- TIDL API (included in TI Processor SDK)
- OpenCL runtime (optional)

## Getting Started

1. Build the module:
   ```bash
   cd td3learn
   mkdir build && cd build
   cmake ..
   make
   ```

2. Run training simulation:
   ```bash
   ./tools/train --config configs/default.yaml
   ```

3. Deploy on hexapod:
   ```bash
   ./tools/deploy --model models/trained_model.tidl
   ```

## Architecture

The module is structured into several components:

- **Agent**: TD3 algorithm implementation
- **Environment**: Interface with hexapod hardware or simulation
- **Models**: Neural network definitions for actor and critics
- **Acceleration**: TIDL and OpenCL integration for hardware acceleration
- **Utils**: Configuration, logging, and visualization tools

## License

[License information here]
