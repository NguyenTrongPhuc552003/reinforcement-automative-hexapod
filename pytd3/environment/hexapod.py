"""
Hexapod environment interface for reinforcement learning
"""
import time
import numpy as np
import gym
from gym import spaces

class HexapodEnvironment(gym.Env):
    """
    Hexapod environment implementing the OpenAI Gym interface.
    This environment can work with either hardware or simulation.
    """
    
    def __init__(
        self,
        use_hardware=False,
        action_dim=18,
        state_dim=24,
        max_episode_steps=1000,
        reward_weights=None,
        time_step=0.05
    ):
        """
        Initialize the hexapod environment.
        
        Args:
            use_hardware: Whether to use actual hardware or simulation
            action_dim: Dimension of the action space (typically 18 for 6 legs × 3 joints)
            state_dim: Dimension of the state space
            max_episode_steps: Maximum steps per episode
            reward_weights: Weights for different reward components
            time_step: Time step between actions (seconds)
        """
        super(HexapodEnvironment, self).__init__()
        
        self.use_hardware = use_hardware
        self.action_dim = action_dim
        self.state_dim = state_dim
        self.max_episode_steps = max_episode_steps
        self.time_step = time_step
        
        # Define default reward weights if none provided
        self.reward_weights = reward_weights or {
            'stability': 1.0,
            'energy': 0.5,
            'velocity': 0.8,
            'direction': 0.6,
            'smoothness': 0.4
        }
        
        # Define action and observation spaces
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(action_dim,), dtype=np.float32
        )
        
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(state_dim,), dtype=np.float32
        )
        
        # Initialize state and episode counters
        self.current_state = np.zeros(state_dim, dtype=np.float32)
        self.steps = 0
        self.episode = 0
        self.last_action = np.zeros(action_dim, dtype=np.float32)
        
        # Initialize hardware interface if using hardware
        self.hardware = None
        if use_hardware:
            self._setup_hardware()
    
    def _setup_hardware(self):
        """Set up hardware interface"""
        try:
            from pytd3.bridge.client import HexapodHardwareClient
            self.hardware = HexapodHardwareClient()
            success = self.hardware.connect()
            if not success:
                print("WARNING: Failed to connect to hexapod hardware. Using simulation instead.")
                self.use_hardware = False
                self.hardware = None
        except ImportError as e:
            print(f"WARNING: Hardware interface not available: {e}")
            self.use_hardware = False
            self.hardware = None
    
    def reset(self):
        """
        Reset the environment to initial state.
        
        Returns:
            Initial observation
        """
        self.steps = 0
        self.episode += 1
        
        # Reset hardware if using actual hardware
        if self.use_hardware and self.hardware:
            self.hardware.reset()
        
        # Generate initial state
        self.current_state = self._get_observation()
        self.last_action = np.zeros(self.action_dim, dtype=np.float32)
        
        return self.current_state
    
    def step(self, action):
        """
        Take a step in the environment.
        
        Args:
            action: Action to take
            
        Returns:
            observation: New state observation
            reward: Reward for the action
            done: Whether the episode is finished
            info: Additional information
        """
        # Increment step counter
        self.steps += 1
        
        # Clip action to valid range
        action = np.clip(action, -1.0, 1.0)
        
        # Apply action to hardware or simulation
        if self.use_hardware and self.hardware:
            self.hardware.apply_action(action)
        else:
            # Simulate action in software
            self._simulate_action(action)
        
        # Save action for smoothness calculations
        self.last_action = action.copy()
        
        # Get new state after action
        self.current_state = self._get_observation()
        
        # Calculate reward
        reward = self._calculate_reward(action)
        
        # Check if episode is done
        done = self.steps >= self.max_episode_steps
        
        # Additional info
        info = {
            'steps': self.steps,
            'episode': self.episode
        }
        
        return self.current_state, reward, done, info
    
    def _get_observation(self):
        """
        Get current observation (state).
        
        Returns:
            Current state observation
        """
        if self.use_hardware and self.hardware:
            # Get state from hardware
            return self.hardware.get_state()
        else:
            # Generate simulated state
            return self._generate_simulated_state()
    
    def _generate_simulated_state(self):
        """
        Generate a simulated state for testing without hardware.
        
        Returns:
            Simulated state vector
        """
        # In a real implementation, this would simulate hexapod physics
        # Here we just create a simple state with some basic features
        
        # Simple simulation for testing
        state = np.zeros(self.state_dim, dtype=np.float32)
        
        # Add IMU-like data (orientation + acceleration)
        state[0:3] = np.random.normal(0, 0.1, 3)  # orientation
        state[3:6] = np.random.normal(0, 0.2, 3)  # acceleration
        
        # Add joint positions (similar to what the hardware would report)
        if hasattr(self, 'last_action') and self.last_action is not None:
            # Map actions to joint positions
            joint_positions = self.last_action.copy()
            state[6:6+self.action_dim] = joint_positions
        
        return state
    
    def _simulate_action(self, action):
        """
        Simulate the effect of an action without hardware.
        
        Args:
            action: Action to simulate
        """
        # In a real implementation, this would update a physics simulation
        # For now, we'll just add a delay to simulate execution time
        time.sleep(self.time_step)
    
    def _calculate_reward(self, action):
        """
        Calculate reward for the current state and action.
        
        Args:
            action: Action taken
            
        Returns:
            Total reward value
        """
        # Initialize reward components
        reward_components = {}
        
        if self.use_hardware and self.hardware:
            # Get reward components from hardware
            reward_components = self.hardware.get_reward_components()
        else:
            # Calculate simulated reward components
            # Stability reward (higher when orientation is level)
            stability = 1.0 - min(1.0, np.sum(np.abs(self.current_state[0:3])) / 3)
            
            # Energy efficiency (penalize high actuator values)
            energy = 1.0 - min(1.0, np.mean(np.abs(action)) / 0.7)
            
            # Velocity reward (simplified for simulation)
            velocity = max(0, np.mean(self.current_state[3:6]))
            
            # Direction reward (simplified)
            direction = 0.5  # Neutral value for simulation
            
            # Smoothness reward (penalize sudden changes)
            if hasattr(self, 'last_action') and self.last_action is not None:
                action_diff = np.abs(action - self.last_action)
                smoothness = 1.0 - min(1.0, np.mean(action_diff) * 5)
            else:
                smoothness = 0.5
            
            reward_components = {
                'stability': stability,
                'energy': energy,
                'velocity': velocity,
                'direction': direction,
                'smoothness': smoothness
            }
        
        # Calculate weighted sum of reward components
        total_reward = 0
        for component, value in reward_components.items():
            if component in self.reward_weights:
                total_reward += value * self.reward_weights[component]
        
        return total_reward
    
    def close(self):
        """Clean up resources"""
        if self.use_hardware and self.hardware:
            self.hardware.disconnect()
            self.hardware = None
    
    def render(self, mode='human'):
        """
        Render the environment.
        Not implemented for hexapod hardware.
        """
        pass