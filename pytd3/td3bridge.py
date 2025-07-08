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
TD3Bridge - Command-line utility for pytd3 with IPC bridge

This script provides a user-friendly interface for managing pytd3 tasks:
- Training new models
- Running trained models
- Operating the IPC bridge between user space applications and pytd3
"""
import os
import sys
import argparse
import logging
from pytd3.main import train, run, bridge_mode

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("td3bridge")

def main():
    """
    Main entry point for TD3Bridge command-line utility
    """
    # Create main parser
    parser = argparse.ArgumentParser(
        description="TD3Bridge - pytd3 with IPC bridge for hexapod robot control"
    )
    
    # Create subparsers for different commands
    subparsers = parser.add_subparsers(dest="command", help="Command to run")
    
    # Train command
    train_parser = subparsers.add_parser("train", help="Train a new model")
    train_parser.add_argument(
        "--config", 
        type=str, 
        default="pytd3/config/default.yaml",
        help="Path to configuration file"
    )
    
    # Run command
    run_parser = subparsers.add_parser("run", help="Run a trained model")
    run_parser.add_argument(
        "--model", 
        type=str, 
        required=True,
        help="Path to trained model file"
    )
    run_parser.add_argument(
        "--config", 
        type=str, 
        default="pytd3/config/default.yaml",
        help="Path to configuration file"
    )
    
    # Bridge command
    bridge_parser = subparsers.add_parser("bridge", help="Run the IPC bridge in standalone mode")
    bridge_parser.add_argument(
        "--model", 
        type=str, 
        required=True,
        help="Path to trained model file"
    )
    bridge_parser.add_argument(
        "--config", 
        type=str, 
        default="pytd3/config/default.yaml",
        help="Path to configuration file"
    )
    
    # Parse arguments
    args = parser.parse_args()
    
    # Execute command
    if args.command == "train":
        logger.info(f"Training new model with config: {args.config}")
        train(args.config)
    elif args.command == "run":
        logger.info(f"Running model {args.model} with config: {args.config}")
        run(args.model, args.config)
    elif args.command == "bridge":
        logger.info(f"Starting bridge with model {args.model} and config: {args.config}")
        bridge_mode(args.model, args.config)
    else:
        parser.print_help()
        return 1
        
    return 0

if __name__ == "__main__":
    sys.exit(main())