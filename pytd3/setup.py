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