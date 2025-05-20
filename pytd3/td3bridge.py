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