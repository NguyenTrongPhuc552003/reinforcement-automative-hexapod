"""
Client interface to communicate with hexapod hardware.
"""
import socket
import json
import time
import logging
import numpy as np

logger = logging.getLogger(__name__)

class HexapodHardwareClient:
    """
    Client interface to communicate with hexapod hardware.
    Uses TCP sockets to send commands and receive state.
    """
    
    def __init__(self, host='localhost', port=8080, timeout=5.0):
        """
        Initialize the hexapod hardware client.
        
        Args:
            host: Hostname or IP of hexapod control server
            port: Port of hexapod control server
            timeout: Connection timeout in seconds
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None
        self.connected = False
    
    def connect(self):
        """
        Connect to hexapod control server.
        
        Returns:
            True if connection successful
        """
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.host, self.port))
            self.connected = True
            logger.info(f"Connected to hexapod server at {self.host}:{self.port}")
            return True
        except socket.error as e:
            logger.error(f"Failed to connect to hexapod server: {e}")
            self.sock = None
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from hexapod control server"""
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception as e:
                logger.warning(f"Error while disconnecting: {e}")
            finally:
                self.sock = None
                self.connected = False
    
    def _send_command(self, command, data=None, retry=1):
        """
        Send command to hexapod server.
        
        Args:
            command: Command name
            data: Command data
            retry: Number of retries
            
        Returns:
            Server response or None
        """
        if not self.connected:
            logger.warning("Not connected to hexapod server")
            return None
        
        # Prepare message
        message = {
            'cmd': command,
            'data': data or {}
        }
        
        # Convert to JSON
        message_str = json.dumps(message)
        
        # Try to send command
        for attempt in range(retry + 1):
            try:
                # Send message
                self.sock.sendall(message_str.encode('utf-8') + b'\n')
                
                # Receive response
                response = b''
                while b'\n' not in response:
                    chunk = self.sock.recv(4096)
                    if not chunk:
                        break
                    response += chunk
                
                if not response:
                    logger.warning(f"No response received (attempt {attempt+1}/{retry+1})")
                    if attempt < retry:
                        time.sleep(0.5)
                        continue
                    return None
                
                # Parse response
                response_str = response.decode('utf-8').strip()
                response_data = json.loads(response_str)
                return response_data
            
            except (socket.error, json.JSONDecodeError) as e:
                logger.warning(f"Error in communication (attempt {attempt+1}/{retry+1}): {e}")
                if attempt < retry:
                    time.sleep(0.5)
                    # Try to reconnect
                    self.disconnect()
                    if not self.connect():
                        return None
                else:
                    return None
        
        return None
    
    def reset(self):
        """
        Reset hexapod to default position.
        
        Returns:
            True if reset successful
        """
        response = self._send_command('reset')
        if response and response.get('status') == 'ok':
            return True
        return False
    
    def apply_action(self, action):
        """
        Apply action to hexapod.
        
        Args:
            action: Action array to apply
            
        Returns:
            True if action applied successfully
        """
        # Convert action to list for JSON serialization
        action_list = action.tolist() if hasattr(action, 'tolist') else list(action)
        
        # Send action command
        response = self._send_command('action', {'values': action_list})
        if response and response.get('status') == 'ok':
            return True
        return False
    
    def get_state(self):
        """
        Get current hexapod state.
        
        Returns:
            State array or zeros if failed
        """
        response = self._send_command('state')
        if response and response.get('status') == 'ok' and 'state' in response:
            # Convert state to numpy array
            return np.array(response['state'], dtype=np.float32)
        
        # Return zeros if failed
        return np.zeros(24, dtype=np.float32)
    
    def get_reward_components(self):
        """
        Get reward components from hexapod.
        
        Returns:
            Dictionary of reward components or empty dict if failed
        """
        response = self._send_command('reward')
        if response and response.get('status') == 'ok' and 'components' in response:
            return response['components']
        
        # Return empty dict if failed
        return {}
