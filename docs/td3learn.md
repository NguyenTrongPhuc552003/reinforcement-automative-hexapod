# TD3Learn: Reinforcement Learning for Hexapod

This guide explains how to build, train, test, and deploy the TD3Learn reinforcement learning system for hexapod locomotion.

## What is TD3Learn?

TD3Learn is an implementation of the Twin Delayed Deep Deterministic Policy Gradient (TD3) reinforcement learning algorithm specifically designed for training hexapod robot locomotion. It enables the robot to learn efficient walking gaits and adapt to different terrains through trial and error.

## Prerequisites

- Docker installed (recommended for consistent environment)
- BeagleBone AI board for hardware deployment
- Basic understanding of reinforcement learning concepts

## Quick Start

Use our helper script to easily build, train, test and deploy TD3Learn models:

```bash
# Build the TD3Learn module
./tools/td3learn_test.sh build

# Train a model for 200 epochs
./tools/td3learn_test.sh train -m my_first_model -e 200

# Test your trained model in simulation
./tools/td3learn_test.sh test -m my_first_model

# Deploy to hardware with TIDL acceleration
./tools/td3learn_test.sh deploy -m my_first_model --tidl --hardware
```

## Understanding the Training Process

### Training Parameters

The training parameters are defined in YAML configuration files:

- `state_dim`: State dimension (typically 24 for hexapod - IMU data and leg positions)
- `action_dim`: Action dimension (typically 18 for hexapod - 3 joints × 6 legs)
- `hidden_layers`: Neural network architecture for actor and critic networks
- `gamma`: Discount factor for future rewards (typically 0.99)
- `batch_size`: Number of experiences to sample for each update
- `buffer_size`: Size of the replay buffer for experience replay

### Reward Function

The default reward function considers:

1. Forward velocity (higher is better)
2. Energy efficiency (lower joint movements are better)
3. Stability (keeping the body level is better)
4. Smoothness (smooth transitions between movements are better)

You can adjust the weights of these components in the config.

### Training Visualization

During training, the system logs:
- Average reward per epoch
- Critic loss
- Actor loss

These metrics are displayed in real-time and saved to log files.

## Testing and Evaluation

When testing a model, the script will:

1. Load your trained model
2. Run it for the specified number of episodes
3. Calculate average reward, stability metrics, and energy efficiency
4. Display the results and save them to a CSV file

## Hardware Deployment

For deployment on the BeagleBone AI:

1. The model is converted to an optimized format
2. If TIDL is enabled, additional optimizations for BeagleBone AI accelerators are applied
3. The model is transferred to the robot
4. A run script is generated to execute the model

## Advanced Usage

### Custom Reward Functions

You can customize the reward function by modifying the `calculateReward` method in the HexapodEnvironment class.

### Hyperparameter Tuning

For better performance, try adjusting:
- Learning rates
- Network architecture
- Exploration noise parameters
- Batch size and buffer size

### Hardware Acceleration

TD3Learn supports two hardware acceleration options:
- TIDL: TI Deep Learning library for EVE and DSP acceleration
- OpenCL: General-purpose GPU acceleration

## Troubleshooting

Common issues:

1. **Low rewards during training**: Try increasing exploration noise or adjusting the reward function
2. **Poor real-world performance**: Simulation to reality gap - try domain randomization
3. **TIDL conversion failures**: Check input dimensions match the model architecture
4. **Memory issues**: Reduce batch size or model complexity
