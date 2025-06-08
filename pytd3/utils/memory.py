"""
Replay buffer implementation for TD3 agent.
"""
import numpy as np
import torch

class ReplayBuffer:
    """
    Replay buffer for storing and sampling experiences.
    """
    
    def __init__(self, state_dim, action_dim, max_size=1_000_000, device="cpu"):
        """
        Initialize the replay buffer.
        
        Args:
            state_dim: Dimension of state space
            action_dim: Dimension of action space
            max_size: Maximum buffer capacity
            device: Device to use for tensor operations
        """
        self.max_size = max_size
        self.ptr = 0
        self.size = 0
        self.device = device
        
        # Create storage arrays
        self.state = np.zeros((max_size, state_dim), dtype=np.float32)
        self.action = np.zeros((max_size, action_dim), dtype=np.float32)
        self.next_state = np.zeros((max_size, state_dim), dtype=np.float32)
        self.reward = np.zeros((max_size, 1), dtype=np.float32)
        self.not_done = np.zeros((max_size, 1), dtype=np.float32)
    
    def add(self, state, action, next_state, reward, done):
        """
        Add a transition to the buffer.
        
        Args:
            state: Current state
            action: Action taken
            next_state: Next state
            reward: Reward received
            done: Whether episode is done
        """
        # Add transition at current pointer position
        self.state[self.ptr] = state
        self.action[self.ptr] = action
        self.next_state[self.ptr] = next_state
        self.reward[self.ptr] = reward
        self.not_done[self.ptr] = 1.0 - float(done)
        
        # Update pointer and size
        self.ptr = (self.ptr + 1) % self.max_size
        self.size = min(self.size + 1, self.max_size)
    
    def sample(self, batch_size):
        """
        Sample a batch of transitions.
        
        Args:
            batch_size: Number of transitions to sample
            
        Returns:
            Batch of transitions as tensors
        """
        # Sample random indices
        ind = np.random.randint(0, self.size, size=batch_size)
        
        # Return batch as tensors on specified device
        return (
            torch.FloatTensor(self.state[ind]).to(self.device),
            torch.FloatTensor(self.action[ind]).to(self.device),
            torch.FloatTensor(self.next_state[ind]).to(self.device),
            torch.FloatTensor(self.reward[ind]).to(self.device),
            torch.FloatTensor(self.not_done[ind]).to(self.device)
        )
    
    def save(self, filename):
        """
        Save buffer to file.
        
        Args:
            filename: File path to save to
        """
        np.savez_compressed(
            filename,
            state=self.state[:self.size],
            action=self.action[:self.size],
            next_state=self.next_state[:self.size],
            reward=self.reward[:self.size],
            not_done=self.not_done[:self.size],
            ptr=self.ptr,
            size=self.size
        )
    
    def load(self, filename):
        """
        Load buffer from file.
        
        Args:
            filename: File path to load from
            
        Returns:
            True if load successful
        """
        try:
            data = np.load(filename)
            
            # Get actual size of saved data
            saved_size = data['size']
            
            # Ensure we don't exceed buffer capacity
            load_size = min(saved_size, self.max_size)
            
            # Load data
            self.state[:load_size] = data['state'][:load_size]
            self.action[:load_size] = data['action'][:load_size]
            self.next_state[:load_size] = data['next_state'][:load_size]
            self.reward[:load_size] = data['reward'][:load_size]
            self.not_done[:load_size] = data['not_done'][:load_size]
            
            # Set size and pointer
            self.size = load_size
            self.ptr = load_size % self.max_size
            
            return True
        except (IOError, ValueError) as e:
            print(f"Error loading buffer: {e}")
            return False