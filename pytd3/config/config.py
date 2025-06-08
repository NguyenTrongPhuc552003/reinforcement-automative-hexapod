"""
Configuration management for pytd3
"""
import os
import yaml
import logging
from dataclasses import dataclass, field
from typing import Dict, Optional, List, Any

@dataclass
class AgentConfig:
    """TD3 agent configuration"""
    hidden_dim: int = 256
    discount: float = 0.99
    tau: float = 0.005  # Target network update rate
    policy_noise: float = 0.2
    noise_clip: float = 0.5
    policy_freq: int = 2
    lr_actor: float = 0.0003
    lr_critic: float = 0.0003

@dataclass
class EnvironmentConfig:
    """Environment configuration"""
    use_hardware: bool = False
    action_dim: int = 18
    state_dim: int = 24
    max_episode_steps: int = 1000
    time_step: float = 0.05
    reward_weights: Dict[str, float] = field(default_factory=lambda: {
        "stability": 1.0,
        "energy": 0.5,
        "progress": 0.8
    })

@dataclass
class TrainingConfig:
    """Training configuration"""
    max_timesteps: int = 1000000
    start_timesteps: int = 25000
    batch_size: int = 256
    exploration_noise: float = 0.1
    buffer_size: int = 1000000
    eval_freq: int = 5000
    save_freq: int = 10000

@dataclass
class BridgeConfig:
    """IPC bridge configuration"""
    enabled: bool = True
    shared_memory_key: int = 1234
    sampling_rate_hz: int = 20
    timeout_ms: int = 100

@dataclass
class ModelConfig:
    """Model configuration"""
    save_dir: str = "/app/models"
    load_model: str = ""

@dataclass
class Config:
    """Overall configuration"""
    agent: AgentConfig = field(default_factory=AgentConfig)
    environment: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    training: TrainingConfig = field(default_factory=TrainingConfig)
    bridge: BridgeConfig = field(default_factory=BridgeConfig)
    model: ModelConfig = field(default_factory=ModelConfig)

def load_config(config_file: str) -> Config:
    """
    Load configuration from YAML file
    
    Args:
        config_file: Path to YAML configuration file
        
    Returns:
        Config object
    """
    config = Config()
    
    if not os.path.exists(config_file):
        logging.warning(f"Config file {config_file} not found, using defaults")
        return config
        
    try:
        with open(config_file, 'r') as f:
            yaml_config = yaml.safe_load(f)
            
        # Update agent config
        if 'agent' in yaml_config:
            for key, value in yaml_config['agent'].items():
                if hasattr(config.agent, key):
                    setattr(config.agent, key, value)
        
        # Update environment config
        if 'environment' in yaml_config:
            for key, value in yaml_config['environment'].items():
                if hasattr(config.environment, key):
                    setattr(config.environment, key, value)
        
        # Update training config
        if 'training' in yaml_config:
            for key, value in yaml_config['training'].items():
                if hasattr(config.training, key):
                    setattr(config.training, key, value)
        
        # Update bridge config
        if 'bridge' in yaml_config:
            for key, value in yaml_config['bridge'].items():
                if hasattr(config.bridge, key):
                    setattr(config.bridge, key, value)
                    
        # Update model config
        if 'model' in yaml_config:
            for key, value in yaml_config['model'].items():
                if hasattr(config.model, key):
                    setattr(config.model, key, value)
        
        return config
        
    except Exception as e:
        logging.error(f"Error loading config from {config_file}: {e}")
        return config

def save_config(config: Config, config_file: str) -> bool:
    """
    Save configuration to YAML file
    
    Args:
        config: Config object
        config_file: Path to save YAML configuration
        
    Returns:
        True if successful
    """
    try:
        # Create config dict
        config_dict = {
            'agent': {k: v for k, v in config.agent.__dict__.items()},
            'environment': {k: v for k, v in config.environment.__dict__.items()},
            'training': {k: v for k, v in config.training.__dict__.items()},
            'bridge': {k: v for k, v in config.bridge.__dict__.items()},
            'model': {k: v for k, v in config.model.__dict__.items()}
        }
        
        # Create directory if it doesn't exist
        os.makedirs(os.path.dirname(config_file), exist_ok=True)
        
        # Write to file
        with open(config_file, 'w') as f:
            yaml.dump(config_dict, f, default_flow_style=False)
            
        return True
        
    except Exception as e:
        logging.error(f"Error saving config to {config_file}: {e}")
        return False