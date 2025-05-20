"""
TD3 main module for reinforcement learning with IPC bridge
"""
import os
import sys
import time
import argparse
import torch
import numpy as np
import logging
from typing import Dict, Any, Optional, List, Tuple

# Import components
from pytd3.agent.td3 import TD3Agent
from pytd3.environment.hexapod import HexapodEnvironment
from pytd3.utils.memory import ReplayBuffer
from pytd3.bridge.ipc import TD3Bridge, TD3ProcessBridge
from pytd3.config.config import Config

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("pytd3")

def train(config_path: str) -> None:
    """
    Train a TD3 model
    
    Args:
        config_path: Path to the configuration file
    """
    # Load configuration
    cfg = Config(config_path)
    if not cfg.validate():
        logger.error("Invalid configuration")
        return
        
    # Create environment
    env_config = cfg.get_section("environment")
    env = HexapodEnvironment(
        use_hardware=env_config.get("use_hardware", False),
        action_dim=env_config.get("action_dim", 18),
        state_dim=env_config.get("state_dim", 24),
        max_episode_steps=env_config.get("max_episode_steps", 1000),
        reward_weights=env_config.get("reward_weights", None),
        time_step=env_config.get("time_step", 0.05)
    )
    
    # Get dimensions
    state_dim = env.state_dim
    action_dim = env.action_dim
    max_action = 1.0  # Environment assumes normalized actions in [-1, 1]
    
    # Update configuration with actual dimensions
    cfg.set("agent", "state_dim", state_dim)
    cfg.set("agent", "action_dim", action_dim)
    cfg.set("agent", "max_action", max_action)
    
    # Set device (CPU/GPU)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logger.info(f"Using device: {device}")
    
    # Create agent
    agent_config = cfg.get_section("agent")
    agent = TD3Agent(
        state_dim=state_dim,
        action_dim=action_dim,
        max_action=max_action,
        device=device,
        hidden_dim=agent_config.get("hidden_dim", 256),
        discount=agent_config.get("discount", 0.99),
        tau=agent_config.get("tau", 0.005),
        policy_noise=agent_config.get("policy_noise", 0.2),
        noise_clip=agent_config.get("noise_clip", 0.5),
        policy_freq=agent_config.get("policy_freq", 2),
        lr_actor=agent_config.get("lr_actor", 3e-4),
        lr_critic=agent_config.get("lr_critic", 3e-4)
    )
    
    # Create replay buffer
    training_config = cfg.get_section("training")
    replay_buffer = ReplayBuffer(
        state_dim=state_dim,
        action_dim=action_dim,
        max_size=training_config.get("buffer_size", 1_000_000),
        device=device
    )
    
    # Create IPC bridge if enabled
    bridge = None
    if cfg.get("bridge", "enabled", True):
        bridge_config = cfg.get_section("bridge")
        bridge = TD3Bridge(
            socket_path=bridge_config.get("socket_path", "/tmp/td3_bridge.sock"),
            buffer_size=bridge_config.get("buffer_size", 4096)
        )
        if not bridge.start():
            logger.error("Failed to start IPC bridge")
            return
    
    # Training parameters
    max_timesteps = training_config.get("max_timesteps", 1_000_000) 
    start_timesteps = training_config.get("start_timesteps", 25_000)
    batch_size = training_config.get("batch_size", 256)
    exploration_noise = training_config.get("exploration_noise", 0.1)
    eval_freq = training_config.get("eval_freq", 5_000)
    save_freq = training_config.get("save_freq", 10_000)
    
    # Create model directory if it doesn't exist
    model_dir = cfg.get("model", "save_dir", "models")
    os.makedirs(model_dir, exist_ok=True)
    
    # Load model if specified
    model_path = cfg.get("model", "load_model", "")
    if model_path and os.path.isfile(model_path):
        logger.info(f"Loading model from {model_path}")
        agent.load(model_path)
    
    # Training loop
    state, done = env.reset(), False
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0
    
    logger.info("Starting training")
    
    for t in range(1, max_timesteps + 1):
        episode_timesteps += 1
        
        # Select action based on current policy or random for exploration
        if t < start_timesteps:
            # Random action for exploration at the beginning
            action = np.random.uniform(
                -max_action, max_action, size=action_dim
            )
        else:
            # Use policy with exploration noise
            action = agent.select_action(state, add_noise=True, noise_std=exploration_noise)
        
        # Take action in the environment
        next_state, reward, done, info = env.step(action)
        
        # Store transition in the replay buffer
        replay_buffer.add(state, action, next_state, reward, done)
        
        # Update state and metrics
        state = next_state
        episode_reward += reward
        
        # Train agent after collecting enough samples
        if t >= start_timesteps:
            train_metrics = agent.train(replay_buffer, batch_size)
            
            # Log training metrics occasionally
            if t % 1000 == 0:
                logger.info(f"Training iteration {t}: critic_loss={train_metrics['critic_loss']:.4f}, actor_loss={train_metrics['actor_loss']:.4f}")
        
        # If episode is done
        if done or episode_timesteps >= env_config.get("max_episode_steps", 1000):
            logger.info(f"Episode {episode_num + 1}: {episode_reward:.2f} reward, {episode_timesteps} steps")
            
            # Reset for next episode
            state, done = env.reset(), False
            episode_reward = 0
            episode_timesteps = 0
            episode_num += 1
        
        # Evaluate and save the model
        if t % eval_freq == 0:
            evaluate(agent, env, training_config.get("eval_episodes", 10))
            
        if t % save_freq == 0:
            model_file = os.path.join(model_dir, f"td3_model_{t}.pt")
            agent.save(model_file)
            logger.info(f"Model saved to {model_file}")
    
    # Final save
    model_file = os.path.join(model_dir, "td3_model_final.pt")
    agent.save(model_file)
    logger.info(f"Training complete. Final model saved to {model_file}")
    
    # Clean up
    env.close()
    if bridge:
        bridge.stop()

def evaluate(agent: TD3Agent, env: HexapodEnvironment, eval_episodes: int = 10) -> float:
    """
    Evaluate the agent
    
    Args:
        agent: TD3 agent
        env: Environment
        eval_episodes: Number of episodes to evaluate
        
    Returns:
        Average reward
    """
    logger.info(f"Evaluating for {eval_episodes} episodes")
    
    rewards = []
    
    for episode in range(eval_episodes):
        state, done = env.reset(), False
        episode_reward = 0
        
        while not done:
            # Select action without exploration noise
            action = agent.select_action(state, add_noise=False)
            
            # Take action in the environment
            state, reward, done, _ = env.step(action)
            episode_reward += reward
        
        rewards.append(episode_reward)
    
    avg_reward = np.mean(rewards)
    logger.info(f"Evaluation: {avg_reward:.2f} average reward over {eval_episodes} episodes")
    
    return avg_reward

def run(model_path: str, config_path: str) -> None:
    """
    Run a trained TD3 model
    
    Args:
        model_path: Path to the model file
        config_path: Path to the configuration file
    """
    # Load configuration
    cfg = Config(config_path)
    if not cfg.validate():
        logger.error("Invalid configuration")
        return
    
    # Create environment
    env_config = cfg.get_section("environment")
    env = HexapodEnvironment(
        use_hardware=env_config.get("use_hardware", False),
        action_dim=env_config.get("action_dim", 18),
        state_dim=env_config.get("state_dim", 24),
        max_episode_steps=env_config.get("max_episode_steps", 1000),
        reward_weights=env_config.get("reward_weights", None),
        time_step=env_config.get("time_step", 0.05)
    )
    
    # Get dimensions
    state_dim = env.state_dim
    action_dim = env.action_dim
    max_action = 1.0
    
    # Set device (CPU/GPU)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logger.info(f"Using device: {device}")
    
    # Create agent
    agent_config = cfg.get_section("agent")
    agent = TD3Agent(
        state_dim=state_dim,
        action_dim=action_dim,
        max_action=max_action,
        device=device,
        hidden_dim=agent_config.get("hidden_dim", 256)
    )
    
    # Load model
    if not os.path.isfile(model_path):
        logger.error(f"Model file not found: {model_path}")
        return
        
    logger.info(f"Loading model from {model_path}")
    agent.load(model_path)
    
    # Create IPC bridge if enabled
    bridge = None
    if cfg.get("bridge", "enabled", True):
        bridge_config = cfg.get_section("bridge")
        bridge = TD3Bridge(
            socket_path=bridge_config.get("socket_path", "/tmp/td3_bridge.sock"),
            buffer_size=bridge_config.get("buffer_size", 4096)
        )
        if not bridge.start():
            logger.error("Failed to start IPC bridge")
            return
    
    # Run the agent
    logger.info("Running the agent")
    
    try:
        state, done = env.reset(), False
        episode_reward = 0
        episode_timesteps = 0
        episode_num = 0
        
        while True:
            episode_timesteps += 1
            
            # Select action using the policy
            action = agent.select_action(state)
            
            # Share action through bridge if available
            if bridge:
                bridge.send_action(action)
            
            # Take action in the environment
            next_state, reward, done, info = env.step(action)
            episode_reward += reward
            
            # Update state
            state = next_state
            
            # If episode is done
            if done or episode_timesteps >= env_config.get("max_episode_steps", 1000):
                logger.info(f"Episode {episode_num + 1}: {episode_reward:.2f} reward, {episode_timesteps} steps")
                
                # Reset for next episode
                state, done = env.reset(), False
                episode_reward = 0
                episode_timesteps = 0
                episode_num += 1
                
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    
    # Clean up
    env.close()
    if bridge:
        bridge.stop()

def bridge_process(
    state_array, action_array, reward_value, done_flag,
    state_ready, action_ready, feedback_ready, exit_flag, 
    agent_args
):
    """
    Agent process function for multi-process bridge
    
    Args:
        state_array: Shared memory for states
        action_array: Shared memory for actions
        reward_value: Shared memory for rewards
        done_flag: Shared memory for done flag
        state_ready: Event to signal state is ready
        action_ready: Event to signal action is ready
        feedback_ready: Event to signal feedback is ready
        exit_flag: Flag to signal process should exit
        agent_args: Arguments for the agent
    """
    try:
        # Create agent
        device = torch.device("cuda" if torch.cuda.is_available() and agent_args.get("use_cuda", True) else "cpu")
        
        state_dim = len(state_array)
        action_dim = len(action_array)
        max_action = agent_args.get("max_action", 1.0)
        
        agent = TD3Agent(
            state_dim=state_dim,
            action_dim=action_dim,
            max_action=max_action,
            device=device,
            hidden_dim=agent_args.get("hidden_dim", 256)
        )
        
        # Load model if specified
        model_path = agent_args.get("model_path", "")
        if model_path and os.path.isfile(model_path):
            agent.load(model_path)
        
        # Process loop
        while True:
            # Check exit flag
            with exit_flag.get_lock():
                if exit_flag.value:
                    break
            
            # Wait for state
            state_ready.wait()
            state_ready.clear()
            
            # Check exit flag after waking up
            with exit_flag.get_lock():
                if exit_flag.value:
                    break
            
            # Get state from shared memory
            with state_array.get_lock():
                state = np.array([state_array[i] for i in range(state_dim)])
            
            # Select action
            action = agent.select_action(state)
            
            # Store action in shared memory
            with action_array.get_lock():
                for i, val in enumerate(action):
                    action_array[i] = val
            
            # Signal that action is ready
            action_ready.set()
            
            # Wait for feedback (optional)
            if feedback_ready.wait(timeout=1.0):
                feedback_ready.clear()
                
                # Get reward and done flag
                with reward_value.get_lock(), done_flag.get_lock():
                    reward = reward_value.value
                    done = done_flag.value
                
                # No training in this process, just acknowledge feedback
                pass
                
    except Exception as e:
        logger.error(f"Error in bridge process: {e}")
    
    logger.info("Bridge process exiting")

def bridge_mode(model_path: str, config_path: str) -> None:
    """
    Run in bridge-only mode
    
    Args:
        model_path: Path to the model file
        config_path: Path to the configuration file
    """
    # Load configuration
    cfg = Config(config_path)
    if not cfg.validate():
        logger.error("Invalid configuration")
        return
    
    # Get agent and bridge configuration
    agent_config = cfg.get_section("agent")
    bridge_config = cfg.get_section("bridge")
    
    # Create multi-process bridge
    state_dim = agent_config.get("state_dim", 24)
    action_dim = agent_config.get("action_dim", 18)
    
    process_bridge = TD3ProcessBridge(state_dim, action_dim)
    
    # Create agent arguments
    agent_args = {
        "model_path": model_path,
        "hidden_dim": agent_config.get("hidden_dim", 256),
        "max_action": agent_config.get("max_action", 1.0),
        "use_cuda": torch.cuda.is_available()
    }
    
    # Start the agent process
    if not process_bridge.start_agent_process(bridge_process, agent_args):
        logger.error("Failed to start agent process")
        return
    
    # Create IPC bridge for communication with user space applications
    ipc_bridge = TD3Bridge(
        socket_path=bridge_config.get("socket_path", "/tmp/td3_bridge.sock"),
        buffer_size=bridge_config.get("buffer_size", 4096)
    )
    if not ipc_bridge.start():
        logger.error("Failed to start IPC bridge")
        process_bridge.stop_agent_process()
        return
    
    # Register handlers for the IPC bridge
    def handle_get_action(data):
        state = np.array(data.get("state", []))
        if len(state) != state_dim:
            return {"id": data.get("id", 0), "status": "error", "message": "Invalid state dimension"}
        
        # Send state to agent process
        process_bridge.send_state(state)
        
        # Get action from agent process
        action = process_bridge.get_action(timeout=1.0)
        if action is None:
            return {"id": data.get("id", 0), "status": "error", "message": "Timeout waiting for action"}
        
        return {"id": data.get("id", 0), "status": "ok", "action": action.tolist()}
    
    def handle_send_feedback(data):
        reward = data.get("reward", 0.0)
        done = data.get("done", False)
        
        # Send feedback to agent process
        process_bridge.send_feedback(reward, done)
        
        return {"id": data.get("id", 0), "status": "ok"}
    
    # Register handlers
    ipc_bridge.register_handler("get_action", handle_get_action)
    ipc_bridge.register_handler("send_feedback", handle_send_feedback)
    
    logger.info("Bridge mode started, waiting for connections")
    
    try:
        while True:
            # Just keep the main thread alive
            time.sleep(1.0)
            
            # Print status every 10 seconds
            if int(time.time()) % 10 == 0:
                metrics = ipc_bridge.get_performance_metrics()
                logger.info(f"Bridge stats: Requests: {metrics['request_count']}, "
                           f"Avg response time: {metrics['avg_response_time']:.4f}s")
    
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    
    # Clean up
    ipc_bridge.stop()
    process_bridge.stop_agent_process()

def main():
    """
    Main entry point
    """
    parser = argparse.ArgumentParser(description="TD3 reinforcement learning for hexapod robot")
    
    # Common arguments
    parser.add_argument("--config", type=str, default="config/default.yaml", help="Path to configuration file")
    
    # Subparsers for different modes
    subparsers = parser.add_subparsers(dest="mode", help="Operation mode")
    
    # Train mode
    train_parser = subparsers.add_parser("train", help="Train a new model")
    
    # Run mode
    run_parser = subparsers.add_parser("run", help="Run a trained model")
    run_parser.add_argument("--model", type=str, required=True, help="Path to model file")
    
    # Bridge mode
    bridge_parser = subparsers.add_parser("bridge", help="Run in bridge-only mode")
    bridge_parser.add_argument("--model", type=str, required=True, help="Path to model file")
    
    # Parse arguments
    args = parser.parse_args()
    
    # Execute selected mode
    if args.mode == "train":
        train(args.config)
    elif args.mode == "run":
        run(args.model, args.config)
    elif args.mode == "bridge":
        bridge_mode(args.model, args.config)
    else:
        parser.print_help()

if __name__ == "__main__":
    main()