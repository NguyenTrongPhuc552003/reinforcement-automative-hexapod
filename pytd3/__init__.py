"""
PyTD3 - Reinforcement Learning Framework for Hexapod Robot Control

This package provides reinforcement learning capabilities for hexapod control
using the Twin Delayed DDPG (TD3) algorithm with hardware acceleration
on BeagleBone AI.
"""

__version__ = "0.1.0"

# Import main components to make them available at the package level
from .main import train, run, bridge_mode
