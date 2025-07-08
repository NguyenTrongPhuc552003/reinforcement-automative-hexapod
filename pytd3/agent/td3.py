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
Twin Delayed Deep Deterministic Policy Gradient (TD3) agent implementation
"""
import copy
import os
import numpy as np
import torch
import torch.nn.functional as F
from typing import Dict, Any, Tuple, Optional, List

from pytd3.models.actor import Actor
from pytd3.models.critic import Critic
from pytd3.utils.memory import ReplayBuffer

class TD3:
    """Twin Delayed Deep Deterministic Policy Gradient implementation
    
    This implementation follows the TD3 paper: https://arxiv.org/abs/1802.09477
    - Uses twin critics to reduce overestimation bias
    - Delayed policy updates
    - Target policy smoothing
    """
    
    def __init__(
        self,
        state_dim: int,
        action_dim: int,
        max_action: float,
        device: str,
        discount: float = 0.99,
        tau: float = 0.005,
        policy_noise: float = 0.2,
        noise_clip: float = 0.5,
        policy_freq: int = 2,
        actor_lr: float = 3e-4,
        critic_lr: float = 3e-4,
        hidden_dim: int = 256
    ):
        """Initialize TD3 agent"""
        self.device = torch.device(device)
        
        # Initialize actor networks
        self.actor = Actor(state_dim, action_dim, hidden_dim, max_action).to(self.device)
        self.actor_target = copy.deepcopy(self.actor)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=actor_lr)

        # Initialize critic networks (twin critics for TD3)
        self.critic = Critic(state_dim, action_dim, hidden_dim).to(self.device)
        self.critic_target = copy.deepcopy(self.critic)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=critic_lr)
        
        # Algorithm parameters
        self.max_action = max_action
        self.discount = discount
        self.tau = tau
        self.policy_noise = policy_noise * max_action
        self.noise_clip = noise_clip * max_action
        self.policy_freq = policy_freq
        
        # Training step counter (for delayed policy updates)
        self.total_steps = 0
        
    def select_action(self, state: np.ndarray, add_noise: bool = False, noise_scale: float = 0.1) -> np.ndarray:
        """Select an action given the current state"""
        # Convert state to tensor and send to device
        state_tensor = torch.FloatTensor(state.reshape(1, -1)).to(self.device)
        
        # Set actor to evaluation mode
        self.actor.eval()
        
        with torch.no_grad():
            action = self.actor(state_tensor).cpu().data.numpy().flatten()
        
        # Set actor back to training mode
        self.actor.train()
        
        # Add exploration noise if requested
        if add_noise:
            noise = np.random.normal(0, self.max_action * noise_scale, size=action.shape)
            action = np.clip(action + noise, -self.max_action, self.max_action)
            
        return action
    
    def train(self, replay_buffer: ReplayBuffer, batch_size: int = 256) -> Dict[str, float]:
        """Train the agent using a batch of experiences from the replay buffer"""
        self.total_steps += 1
        
        # Sample a batch of transitions from the replay buffer
        state, action, next_state, reward, not_done = replay_buffer.sample(batch_size)
        
        # Convert to tensors and move to device
        state = torch.FloatTensor(state).to(self.device)
        action = torch.FloatTensor(action).to(self.device)
        next_state = torch.FloatTensor(next_state).to(self.device)
        reward = torch.FloatTensor(reward).reshape(-1, 1).to(self.device)
        not_done = torch.FloatTensor(not_done).reshape(-1, 1).to(self.device)
        
        # --- Update critic ---
        with torch.no_grad():
            # Select action according to policy and add clipped noise
            noise = (
                torch.randn_like(action) * self.policy_noise
            ).clamp(-self.noise_clip, self.noise_clip)
            
            next_action = (
                self.actor_target(next_state) + noise
            ).clamp(-self.max_action, self.max_action)
            
            # Compute the target Q value
            target_q1, target_q2 = self.critic_target(next_state, next_action)
            target_q = torch.min(target_q1, target_q2)
            target_q = reward + not_done * self.discount * target_q
            
        # Get current Q estimates
        current_q1, current_q2 = self.critic(state, action)
        
        # Compute critic loss
        critic_loss = F.mse_loss(current_q1, target_q) + F.mse_loss(current_q2, target_q)
        
        # Optimize the critic
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()
        
        # --- Update actor (delayed) ---
        actor_loss = 0.0
        
        if self.total_steps % self.policy_freq == 0:
            # Compute actor loss
            actor_loss = -self.critic.q1(state, self.actor(state)).mean()
            
            # Optimize the actor
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()
            
            # Update the target networks
            self._update_target_networks()
                
        return {
            "critic_loss": critic_loss.item(),
            "actor_loss": actor_loss if isinstance(actor_loss, float) else actor_loss.item()
        }
    
    def _update_target_networks(self):
        """Update target networks with Polyak averaging"""
        for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

        for param, target_param in zip(self.actor.parameters(), self.actor_target.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

    def save(self, path: str) -> None:
        """Save the actor and critic models"""
        # Create directory if it doesn't exist
        os.makedirs(os.path.dirname(path), exist_ok=True)
        
        torch.save(self.actor.state_dict(), f"{path}_actor.pth")
        torch.save(self.critic.state_dict(), f"{path}_critic.pth")
        
    def load(self, path: str) -> None:
        """Load the actor and critic models"""
        self.actor.load_state_dict(torch.load(f"{path}_actor.pth"))
        self.critic.load_state_dict(torch.load(f"{path}_critic.pth"))
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.critic_target.load_state_dict(self.critic.state_dict())