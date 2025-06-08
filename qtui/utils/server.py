#!/usr/bin/env python3
"""
Hexapod Remote Control Server

This server runs on the BeagleBone and provides a network interface to control
the hexapod robot. It communicates with the Qt client application and translates
commands to the C++ hexapod library or simulates responses when in simulation mode.
"""

import os
import sys
import json
import socket
import threading
import time
import logging
import argparse
import ctypes
import math
import errno
from datetime import datetime
from pathlib import Path

# Configure logging
LOG_DIR = "/var/log/hexapod-server"
os.makedirs(LOG_DIR, exist_ok=True)  # Ensure log directory exists

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler(f"{LOG_DIR}/hexapod_server.log", mode='a'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Default configuration
DEFAULT_HOST = '0.0.0.0'  # Listen on all interfaces
DEFAULT_PORT = 8080
DEFAULT_ALT_PORTS = [8081, 8082, 8090, 9090]  # Alternative ports to try if default is unavailable
DEFAULT_LIB_PATH = '/usr/local/lib/libhexapod.so'

class HexapodHardware:
    """Interface to the physical hexapod hardware or simulation"""
    
    def __init__(self, lib_path=DEFAULT_LIB_PATH, simulation=False):
        """Initialize hardware interface"""
        self.simulation = simulation
        self.hexapod = None
        self.hexapod_lib = None
        
        # Simulation state
        self.sim_leg_positions = [{'hip': 0, 'knee': 0, 'ankle': 0} for _ in range(6)]
        self.sim_imu_data = {
            'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 1.0,
            'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0
        }
        
        # Try to load the library if not in simulation mode
        if not simulation:
            try:
                # Try to load the hexapod library
                self.hexapod_lib = ctypes.CDLL(lib_path)
                logger.info(f"Loaded hexapod library from {lib_path}")
                
                # Create hexapod instance
                self.hexapod_lib.Hexapod_create.restype = ctypes.c_void_p
                self.hexapod = self.hexapod_lib.Hexapod_create()
                
                # Initialize hexapod
                self.hexapod_lib.Hexapod_init.argtypes = [ctypes.c_void_p]
                self.hexapod_lib.Hexapod_init.restype = ctypes.c_bool
                if not self.hexapod_lib.Hexapod_init(self.hexapod):
                    logger.error("Failed to initialize hexapod hardware")
                    self.cleanup()
                    self.simulation = True
                else:
                    logger.info("Hexapod hardware initialized successfully")
                    
            except Exception as e:
                logger.error(f"Error initializing hexapod library: {e}")
                logger.info("Falling back to simulation mode")
                self.simulation = True
    
    def cleanup(self):
        """Clean up resources"""
        if not self.simulation and self.hexapod and self.hexapod_lib:
            try:
                # Center all legs in a safe position
                self.hexapod_lib.Hexapod_centerAll.argtypes = [ctypes.c_void_p]
                self.hexapod_lib.Hexapod_centerAll(self.hexapod)
                
                # Clean up resources
                self.hexapod_lib.Hexapod_cleanup.argtypes = [ctypes.c_void_p]
                self.hexapod_lib.Hexapod_cleanup(self.hexapod)
                
                # Destroy instance
                self.hexapod_lib.Hexapod_destroy.argtypes = [ctypes.c_void_p]
                self.hexapod_lib.Hexapod_destroy(self.hexapod)
                
                logger.info("Hexapod hardware cleaned up")
            except Exception as e:
                logger.error(f"Error during hexapod cleanup: {e}")
            
            self.hexapod = None
            self.hexapod_lib = None
    
    def get_imu_data(self):
        """Get IMU sensor data"""
        if self.simulation:
            # In simulation mode, generate simple simulated data
            t = time.time()
            # Add some slow oscillations to simulate movement
            self.sim_imu_data = {
                'accel_x': 0.05 * math.sin(t * 0.5),
                'accel_y': 0.05 * math.cos(t * 0.3),
                'accel_z': 0.98 + 0.02 * math.sin(t * 0.1),
                'gyro_x': 2.0 * math.sin(t * 0.2),
                'gyro_y': 2.0 * math.cos(t * 0.25),
                'gyro_z': 1.0 * math.sin(t * 0.15)
            }
            return self.sim_imu_data
        else:
            try:
                # Create IMU data structure
                class ImuData(ctypes.Structure):
                    _fields_ = [
                        ("accel_x", ctypes.c_int16),
                        ("accel_y", ctypes.c_int16),
                        ("accel_z", ctypes.c_int16),
                        ("gyro_x", ctypes.c_int16),
                        ("gyro_y", ctypes.c_int16),
                        ("gyro_z", ctypes.c_int16)
                    ]
                
                imu_data = ImuData()
                
                # Call library function
                self.hexapod_lib.Hexapod_getImuData.argtypes = [
                    ctypes.c_void_p, ctypes.POINTER(ImuData)
                ]
                self.hexapod_lib.Hexapod_getImuData.restype = ctypes.c_bool
                
                result = self.hexapod_lib.Hexapod_getImuData(
                    self.hexapod, ctypes.byref(imu_data)
                )
                
                if not result:
                    raise RuntimeError("Failed to read IMU data")
                
                # Convert raw data to proper units
                accel_scale = 16384.0  # For ±2g range
                gyro_scale = 65.5  # For ±500°/s range
                
                # Return formatted IMU data
                return {
                    'accel_x': imu_data.accel_x / accel_scale,
                    'accel_y': imu_data.accel_y / accel_scale,
                    'accel_z': imu_data.accel_z / accel_scale,
                    'gyro_x': imu_data.gyro_x / gyro_scale,
                    'gyro_y': imu_data.gyro_y / gyro_scale,
                    'gyro_z': imu_data.gyro_z / gyro_scale
                }
                
            except Exception as e:
                logger.error(f"Error getting IMU data: {e}")
                # Fall back to simulation if hardware fails
                self.simulation = True
                return self.get_imu_data()  # Call again in simulation mode
    
    def set_leg_position(self, leg_num, hip, knee, ankle):
        """Set position for a specific leg"""
        if self.simulation:
            # In simulation mode, just store the values
            if 0 <= leg_num < 6:
                self.sim_leg_positions[leg_num] = {'hip': hip, 'knee': knee, 'ankle': ankle}
            return True
        else:
            try:
                # Call library function
                self.hexapod_lib.Hexapod_setLegPosition.argtypes = [
                    ctypes.c_void_p, ctypes.c_uint8, 
                    ctypes.c_int16, ctypes.c_int16, ctypes.c_int16
                ]
                self.hexapod_lib.Hexapod_setLegPosition.restype = ctypes.c_bool
                
                result = self.hexapod_lib.Hexapod_setLegPosition(
                    self.hexapod, leg_num, hip, knee, ankle
                )
                
                if not result:
                    raise RuntimeError("Failed to set leg position")
                
                return True
                
            except Exception as e:
                logger.error(f"Error setting leg position: {e}")
                return False
    
    def center_all(self):
        """Center all legs to neutral position"""
        if self.simulation:
            # In simulation mode, reset all positions to 0
            for i in range(6):
                self.sim_leg_positions[i] = {'hip': 0, 'knee': 0, 'ankle': 0}
            return True
        else:
            try:
                # Call library function
                self.hexapod_lib.Hexapod_centerAll.argtypes = [ctypes.c_void_p]
                self.hexapod_lib.Hexapod_centerAll.restype = ctypes.c_bool
                
                result = self.hexapod_lib.Hexapod_centerAll(self.hexapod)
                
                if not result:
                    raise RuntimeError("Failed to center all legs")
                
                return True
                
            except Exception as e:
                logger.error(f"Error centering legs: {e}")
                return False
    
    def set_calibration(self, leg_num, hip_offset, knee_offset, ankle_offset):
        """Set calibration offsets for a leg"""
        if self.simulation:
            # In simulation mode, we don't need to do anything
            logger.info(f"Simulation: Set calibration for leg {leg_num}")
            return True
        else:
            try:
                # Call library function
                self.hexapod_lib.Hexapod_setCalibration.argtypes = [
                    ctypes.c_void_p, ctypes.c_uint8, 
                    ctypes.c_int16, ctypes.c_int16, ctypes.c_int16
                ]
                self.hexapod_lib.Hexapod_setCalibration.restype = ctypes.c_bool
                
                result = self.hexapod_lib.Hexapod_setCalibration(
                    self.hexapod, leg_num, hip_offset, knee_offset, ankle_offset
                )
                
                if not result:
                    raise RuntimeError("Failed to set calibration")
                
                return True
                
            except Exception as e:
                logger.error(f"Error setting calibration: {e}")
                return False


class HexapodServer:
    """Server class to handle network connections and control the hexapod"""
    
    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_PORT, lib_path=DEFAULT_LIB_PATH):
        """Initialize server with configuration parameters"""
        self.host = host
        self.port = port
        self.lib_path = lib_path
        self.running = False
        self.server_socket = None
        self.clients = []
        self.final_port = port  # Track the actual port we bind to
        
        # Movement state tracking
        self._current_movement = {'type': 'stop'}
        self._balance_enabled = False
        self._current_tilt = {'pitch': 0.0, 'roll': 0.0}
        
        # Check if we need to run in simulation mode
        simulation = not os.path.exists(lib_path)
        if simulation:
            logger.warning(f"Library {lib_path} not found. Running in simulation mode.")
        
        self.hardware = HexapodHardware(lib_path, simulation)
        
        # Report mode
        if self.hardware.simulation:
            logger.info("Running in SIMULATION mode")
        else:
            logger.info("Running in HARDWARE mode")
    
    def start(self):
        """Start the server"""
        self.running = True
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Set socket timeout to make accept() non-blocking
        self.server_socket.settimeout(1.0)
        
        # Try to bind to the specified port or find an alternative
        ports_to_try = [self.port] + DEFAULT_ALT_PORTS
        bound = False
        
        for port in ports_to_try:
            try:
                logger.info(f"Attempting to bind to {self.host}:{port}...")
                self.server_socket.bind((self.host, port))
                self.final_port = port
                bound = True
                logger.info(f"Successfully bound to {self.host}:{port}")
                break
            except socket.error as e:
                if e.errno == errno.EADDRINUSE:
                    logger.warning(f"Port {port} is already in use, trying next port")
                elif e.errno == errno.EACCES:
                    logger.error(f"Permission denied binding to port {port}. Try using a port > 1024.")
                    break
                else:
                    logger.error(f"Socket error binding to {self.host}:{port}: {e}")
                    if port == self.port:  # Only continue if this was our first choice
                        continue
                    break
        
        if not bound:
            logger.error("Failed to bind to any port. Check network configuration.")
            self.stop()
            return
        
        try:
            self.server_socket.listen(5)
            logger.info(f"Server started on {self.host}:{self.final_port}")
            
            # Accept clients in a loop
            while self.running:
                try:
                    client_socket, client_address = self.server_socket.accept()
                    logger.info(f"New connection from {client_address[0]}:{client_address[1]}")
                    
                    # Start a new thread to handle this client
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(client_socket, client_address)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                    self.clients.append((client_socket, client_thread))
                except socket.timeout:
                    # This is expected due to the timeout we set
                    continue
                except Exception as e:
                    if self.running:
                        logger.error(f"Error accepting client: {e}")
                        # Don't break, try to accept again
                    else:
                        break
        except Exception as e:
            logger.error(f"Server error: {e}", exc_info=True)
        finally:
            self.stop()
    
    def stop(self):
        """Stop the server and clean up resources"""
        if not self.running:
            return  # Already stopped

        self.running = False
        logger.info("Shutting down server...")
        
        # Close all client connections
        for client_socket, _ in self.clients:
            try:
                client_socket.close()
            except Exception as e:
                logger.debug(f"Error closing client socket: {e}")
        
        # Close server socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception as e:
                logger.debug(f"Error closing server socket: {e}")
        
        # Clean up hardware resources
        if self.hardware:
            self.hardware.cleanup()
        
        logger.info("Server stopped")
    
    def handle_client(self, client_socket, client_address):
        """Handle communication with a connected client"""
        buffer = b''
        
        while self.running:
            try:
                # Read data from client
                data = client_socket.recv(1024)
                if not data:
                    break
                
                # Add to buffer and process complete messages
                buffer += data
                
                # Process each complete message (terminated by newline)
                while b'\n' in buffer:
                    pos = buffer.find(b'\n')
                    message = buffer[:pos].decode('utf-8', errors='ignore').strip()
                    buffer = buffer[pos + 1:]
                    
                    # Skip empty messages
                    if not message:
                        continue
                        
                    # Detect and skip HTML or non-JSON content
                    if message.startswith('<') or '<!DOCTYPE' in message or '</html>' in message:
                        logger.warning(f"Received non-JSON data. Skipping.")
                        continue
                    
                    # Process the message
                    try:
                        # Parse JSON message
                        data = json.loads(message)
                        response = self.process_message(data)
                        
                        # Send response back to client
                        client_socket.sendall((json.dumps(response) + '\n').encode('utf-8'))
                    except json.JSONDecodeError as e:
                        logger.error(f"JSON decode error: {e} in message: {message[:100]}")
                        # Send an error response
                        error_response = {
                            'status': 'error',
                            'message': f'Invalid JSON format: {str(e)}'
                        }
                        client_socket.sendall((json.dumps(error_response) + '\n').encode('utf-8'))
                
            except ConnectionResetError:
                logger.info(f"Client {client_address[0]}:{client_address[1]} disconnected")
                break
            except Exception as e:
                logger.error(f"Error handling client {client_address[0]}:{client_address[1]}: {e}")
                break
        
        # Close the connection
        try:
            client_socket.close()
        except:
            pass
        
        logger.info(f"Connection closed for {client_address[0]}:{client_address[1]}")
    
    def process_message(self, message):
        """Process a message from a client"""
        try:
            # Parse JSON message
            data = json.loads(message)
            
            # Check if it's a valid command
            if 'command' not in data:
                return {'status': 'error', 'message': 'Invalid command format'}
            
            command = data['command']
            logger.debug(f"Processing command: {command}")
            
            # Process different command types
            if command == 'ping':
                return {
                    'status': 'success', 
                    'message': 'pong', 
                    'timestamp': int(time.time() * 1000),
                    'simulation': self.hardware.simulation
                }
                
            elif command == 'getImuData':
                imu_data = self.hardware.get_imu_data()
                return {'status': 'success', 'imu_data': imu_data}
                
            elif command == 'setLegPosition':
                leg_num = data.get('leg_num', 0)
                hip = data.get('hip', 0)
                knee = data.get('knee', 0)
                ankle = data.get('ankle', 0)
                
                # Validate parameters
                if not isinstance(leg_num, int) or leg_num < 0 or leg_num >= 6:
                    return {'status': 'error', 'message': 'Invalid leg number'}
                
                result = self.hardware.set_leg_position(leg_num, hip, knee, ankle)
                if result:
                    return {'status': 'success'}
                else:
                    return {'status': 'error', 'message': 'Failed to set leg position'}
                
            elif command == 'centerAll':
                result = self.hardware.center_all()
                if result:
                    return {'status': 'success', 'message': 'All legs centered'}
                else:
                    return {'status': 'error', 'message': 'Failed to center all legs'}
                
            elif command == 'setCalibration':
                leg_num = data.get('leg_num', 0)
                hip_offset = data.get('hip_offset', 0)
                knee_offset = data.get('knee_offset', 0)
                ankle_offset = data.get('ankle_offset', 0)
                
                # Validate parameters
                if not isinstance(leg_num, int) or leg_num < 0 or leg_num >= 6:
                    return {'status': 'error', 'message': 'Invalid leg number'}
                
                result = self.hardware.set_calibration(
                    leg_num, hip_offset, knee_offset, ankle_offset
                )
                if result:
                    return {'status': 'success'}
                else:
                    return {'status': 'error', 'message': 'Failed to set calibration'}
                
            elif command == 'setTilt':
                pitch = data.get('pitch', 0.0)  # Forward/backward tilt
                roll = data.get('roll', 0.0)    # Left/right tilt
                
                self._current_tilt = {'pitch': pitch, 'roll': roll}
                
                # In simulation mode, just acknowledge
                if self.hardware.simulation:
                    return {'status': 'success'}
                
                # In hardware mode, would adjust leg positions to create tilt
                return {'status': 'success'}
                
            elif command == 'setRotation':
                direction = data.get('direction', 0.0)  # -1.0=left, 1.0=right
                speed = data.get('speed', 0.5)          # Speed factor (0.0-1.0)
                
                logger.debug(f"Movement command: direction={direction}, speed={speed}")
                
                self._current_movement = {'type': 'rotation', 'direction': direction, 'speed': speed}
                return {'status': 'success'}
                
            elif command == 'stop':
                logger.info("Stop command received")
                self._current_movement = {'type': 'stop'}
                
                # In simulation mode, just acknowledge
                if self.hardware.simulation:
                    return {'status': 'success', 'message': 'Movement stopped'}
                
                # In hardware mode, center all legs for stability
                result = self.hardware.center_all()
                if result:
                    return {'status': 'success', 'message': 'Movement stopped'}
                else:
                    return {'status': 'error', 'message': 'Failed to stop movement'}
                
            elif command == 'setBalance':
                enabled = data.get('enabled', False)
                self._balance_enabled = enabled
                logger.info(f"Balance mode {'enabled' if enabled else 'disabled'}")
                return {'status': 'success'}
                
            # Unknown command
            return {'status': 'error', 'message': f'Unknown command: {command}'}
            
        except json.JSONDecodeError:
            return {'status': 'error', 'message': 'Invalid JSON format'}
        except Exception as e:
            logger.error(f"Error processing message: {e}")
            return {'status': 'error', 'message': f'Internal server error: {str(e)}'}


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Hexapod Remote Control Server")
    parser.add_argument("--host", default=DEFAULT_HOST, help="Host to listen on")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="Port to listen on")
    parser.add_argument("--lib-path", default=DEFAULT_LIB_PATH, help="Path to hexapod library")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    return parser.parse_args()


def main():
    """Main entry point"""
    args = parse_args()
    
    # Set log level based on arguments
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    logger.info(f"Starting Hexapod Remote Control Server on {args.host}:{args.port}")
    logger.info(f"Using hexapod library: {args.lib_path}")
    
    try:
        # Create and start server
        server = HexapodServer(args.host, args.port, args.lib_path)
        
        # Handle signals for graceful shutdown
        def signal_handler(sig, frame):
            logger.info(f"Received signal {sig}, shutting down server...")
            server.stop()
            sys.exit(0)
        
        # Register signal handlers
        import signal
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Start the server
        server.start()
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Unhandled exception: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()