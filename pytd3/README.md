# TD3 Reinforcement Learning for Hexapod Robot

This directory contains a Twin Delayed Deep Deterministic Policy Gradient (TD3) reinforcement learning implementation for training hexapod robot walking gaits and control strategies.

## Overview

TD3 is an actor-critic reinforcement learning algorithm designed for continuous action spaces. This implementation is specifically adapted for training a hexapod robot to:

- Develop optimal walking gaits
- Navigate terrain efficiently
- Adapt to surface changes
- Respond to obstacles
- Maintain balance and stability

The system bridges between the Python-based TD3 algorithm and the C++ hexapod control framework through a robust communication interface.

## Contents

- **td3bridge.py**: Communication bridge between Python RL algorithms and C++ hexapod controller
- **main.py**: Entry point for training and evaluation scripts
- **setup.py**: Package installation configuration
- **setup.sh**: Environment setup script
- **requirements.txt**: Python dependencies
- **__init__.py**: Package initialization

## Installation

### Prerequisites

- Python 3.8 or higher
- BeagleBone Black with hexapod control software installed
- Virtual environment (recommended)

### Using setup.sh (Recommended)

The quickest way to set up the environment is using the provided setup script:

```bash
# Navigate to pytd3 directory
cd reinforcement-automative-hexapod

# Run setup script
./scripts/build.sh -s
```

This script will:
1. Create a Python virtual environment (if specified)
2. Install required dependencies
3. Configure the Python package
4. Set up communication with the hexapod controller

## Configuration

The TD3 algorithm and bridge interface can be configured through parameters in `main.py`:

- **Learning rate**: Controls how quickly the model adapts (default: 3e-4)
- **Discount factor**: Balance between immediate and future rewards (default: 0.99)
- **Replay buffer size**: Memory for storing experience (default: 1,000,000)
- **Batch size**: Number of samples processed per update (default: 256)
- **Target update rate**: Frequency of target network updates (default: 0.005)
- **Training episodes**: Number of episodes for training (default: 1000)
- **Policy noise**: Exploration noise for policy (default: 0.2)
- **Noise clip**: Maximum noise value (default: 0.5)
- **Policy delay**: Steps between policy updates (default: 2)

## Usage

### Starting Training

To start training a new model:

```bash
# Activate virtual environment if used
source venv/bin/activate

# Start training with default parameters
python main.py --train

# Start training with custom parameters
python main.py --train --episodes 2000 --batch-size 512
```

### Evaluating a Trained Model

To evaluate an existing model:

```bash
# Evaluate the latest model
python main.py --eval

# Evaluate a specific model
python main.py --eval --model-path models/td3_model_1000.pt
```

### Deploying to the Hexapod

To deploy a trained model to the physical hexapod:

```bash
./scripts/deploy.sh
```

## Reward Function

The reinforcement learning agent is guided by a reward function that considers:

1. **Forward velocity**: Positive reward for moving forward
2. **Energy efficiency**: Penalty for excessive joint movements
3. **Stability**: Reward for maintaining level orientation
4. **Smoothness**: Rewards smooth transitions between steps
5. **Task completion**: Bonus rewards for reaching targets

## Bridge Interface

The `td3bridge.py` module provides communication between Python and the C++ hexapod controller:

- **State observations**: Joint positions, IMU data, contact sensors
- **Action execution**: Sending joint commands to the robot
- **Synchronization**: Ensuring timing coordination
- **Error handling**: Robust recovery from communication issues

## Model Architecture

The TD3 implementation uses:

- **Actor network**: 3-layer MLP with ReLU activation
- **Critic networks**: Dual Q-networks for reduced overestimation bias
- **Target networks**: Delayed copies of actor and critics for stability
- **Experience replay**: Buffer storing past experiences for off-policy learning

## Troubleshooting

### Common Issues

1. **Communication errors**:
   - Ensure the hexapod controller is running
   - Check serial port permissions
   - Verify connection settings in td3bridge.py

2. **Training instability**:
   - Reduce learning rate
   - Increase batch size
   - Check reward scaling

3. **Deployment issues**:
   - Ensure model compatibility
   - Verify hardware connections
   - Check error logs at /var/log/hexapod/bridge.log

### Debug Mode

Enable debug logging for more information:

```bash
python main.py --train --debug
```

## Advanced Usage

### Curriculum Learning

Gradually increase task difficulty:

```bash
python main.py --train --curriculum basic,medium,advanced
```

### Hyperparameter Tuning

```bash
python main.py --hyperparam-search
```

### Simulation First Approach

Train in simulation before deploying to hardware:

```bash
python main.py --train --sim-only
python main.py --deploy --sim-to-real
```

## Performance Metrics

The training process outputs several metrics:

- **Average reward**: Overall performance measure
- **Episode length**: Duration of each training episode
- **Success rate**: Percentage of successful task completions
- **Learning curve**: Reward progression over time

Find detailed logs and metrics in the `logs/` directory after training.

## Contributing

When adding new features or fixing bugs:

1. Follow PEP 8 style guidelines
2. Add appropriate tests
3. Update documentation
4. Submit a pull request
