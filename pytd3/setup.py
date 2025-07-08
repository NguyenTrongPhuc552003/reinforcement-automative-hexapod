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

#!/usr/bin/env python3
"""
Setup script for PyTD3
"""
from setuptools import setup, find_packages

setup(
    name="pytd3",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy>=1.16.5",
        "PyYAML>=5.3",
        "pybind11>=2.6.0",
    ],
    author="Hexapod Team",
    author_email="info@hexapod.org",
    description="TD3 Reinforcement Learning for Hexapod Robot Control",
    keywords="robotics, reinforcement learning, TD3, hexapod",
    python_requires=">=3.6",
    entry_points={
        "console_scripts": [
            "td3bridge=pytd3.td3bridge:main",
        ],
    },
)