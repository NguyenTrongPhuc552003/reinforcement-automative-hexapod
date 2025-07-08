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
PyTD3 - Reinforcement Learning Framework for Hexapod Robot Control

This package provides reinforcement learning capabilities for hexapod control
using the Twin Delayed DDPG (TD3) algorithm with hardware acceleration
on BeagleBone AI.
"""

__version__ = "0.1.0"

# Import main components to make them available at the package level
from .main import train, run, bridge_mode
