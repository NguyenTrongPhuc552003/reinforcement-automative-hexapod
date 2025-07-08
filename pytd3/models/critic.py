"""
 * Hexapod Project - A Reinforcement Learning-based Autonomous Hexapod
 * Copyright (C) 2025  Nguyen Trong Phuc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */ """

"""
Critic networks for TD3 agent
"""
import torch
import torch.nn as nn
import torch.nn.functional as F

class Critic(nn.Module):
    """
    Twin critic networks that map state-action pairs to Q-values.
    Used in TD3 to mitigate overestimation bias.
    """
    
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        """
        Initialize twin critic networks.
        
        Args:
            state_dim: Dimension of state space
            action_dim: Dimension of action space
            hidden_dim: Dimension of hidden layers
        """
        super(Critic, self).__init__()
        
        # First critic network
        self.l1 = nn.Linear(state_dim + action_dim, hidden_dim)
        self.l2 = nn.Linear(hidden_dim, hidden_dim)
        self.l3 = nn.Linear(hidden_dim, 1)
        
        # Second critic network
        self.l4 = nn.Linear(state_dim + action_dim, hidden_dim)
        self.l5 = nn.Linear(hidden_dim, hidden_dim)
        self.l6 = nn.Linear(hidden_dim, 1)
        
        # Initialize weights
        self._initialize_weights()
    
    def _initialize_weights(self):
        """Initialize network weights with appropriate scaling"""
        for layer in [self.l1, self.l2, self.l4, self.l5]:
            torch.nn.init.xavier_uniform_(layer.weight)
        
        # Initialize final layer with smaller weights
        for layer in [self.l3, self.l6]:
            torch.nn.init.uniform_(layer.weight, -3e-3, 3e-3)
    
    def Q1(self, state, action):
        """
        Forward pass through the first critic network only.
        Used for policy gradient.
        
        Args:
            state: Input state tensor
            action: Input action tensor
            
        Returns:
            Q-value from the first critic network
        """
        sa = torch.cat([state, action], dim=1)
        
        q1 = F.relu(self.l1(sa))
        q1 = F.relu(self.l2(q1))
        q1 = self.l3(q1)
        
        return q1
    
    def forward(self, state, action):
        """
        Forward pass through both critic networks.
        
        Args:
            state: Input state tensor
            action: Input action tensor
            
        Returns:
            Q-values from both critic networks
        """
        sa = torch.cat([state, action], dim=1)
        
        # First critic
        q1 = F.relu(self.l1(sa))
        q1 = F.relu(self.l2(q1))
        q1 = self.l3(q1)
        
        # Second critic
        q2 = F.relu(self.l4(sa))
        q2 = F.relu(self.l5(q2))
        q2 = self.l6(q2)
        
        return q1, q2