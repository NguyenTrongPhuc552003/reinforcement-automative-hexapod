"""
Actor network for TD3 agent
"""
import torch
import torch.nn as nn
import torch.nn.functional as F

class Actor(nn.Module):
    """
    Actor network that maps states to actions.
    Uses tanh activation to bound actions to [-max_action, max_action].
    """
    
    def __init__(self, state_dim, action_dim, hidden_dim=256, max_action=1.0):
        """
        Initialize the actor network.
        
        Args:
            state_dim: Dimension of state space
            action_dim: Dimension of action space
            hidden_dim: Dimension of hidden layers
            max_action: Maximum action value
        """
        super(Actor, self).__init__()
        
        # Define network layers
        self.l1 = nn.Linear(state_dim, hidden_dim)
        self.l2 = nn.Linear(hidden_dim, hidden_dim)
        self.l3 = nn.Linear(hidden_dim, action_dim)
        
        self.max_action = max_action
        
        # Initialize weights
        self._initialize_weights()
    
    def _initialize_weights(self):
        """Initialize network weights with appropriate scaling"""
        torch.nn.init.xavier_uniform_(self.l1.weight)
        torch.nn.init.xavier_uniform_(self.l2.weight)
        torch.nn.init.uniform_(self.l3.weight, -3e-3, 3e-3)
    
    def forward(self, state):
        """
        Forward pass through the actor network.
        
        Args:
            state: Input state tensor
            
        Returns:
            Action values scaled to [-max_action, max_action]
        """
        a = F.relu(self.l1(state))
        a = F.relu(self.l2(a))
        return self.max_action * torch.tanh(self.l3(a))