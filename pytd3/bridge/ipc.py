"""
IPC (Inter-Process Communication) bridge for TD3 and hexapod control.
"""
import os
import time
import signal
import logging
import multiprocessing as mp
import numpy as np
import ctypes
from shared_memory_dict import SharedMemoryDict

logger = logging.getLogger(__name__)

class TD3Bridge:
    """
    Base class for TD3 communication bridge.
    Handles synchronization between TD3 agent and hexapod control.
    """
    
    def __init__(self, state_dim=24, action_dim=18, buffer_size=5):
        """
        Initialize the TD3 bridge.
        
        Args:
            state_dim: Dimension of the state space
            action_dim: Dimension of the action space
            buffer_size: Size of the buffer for states and actions
        """
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.buffer_size = buffer_size
        
        # Create shared memory for synchronization and data exchange
        self.shared_dict = SharedMemoryDict(name='td3_bridge', size=1024*1024)
        
        # Initialize shared memory dictionary
        if 'initialized' not in self.shared_dict:
            self.shared_dict.update({
                'initialized': False,
                'state': np.zeros(state_dim, dtype=np.float32).tobytes(),
                'action': np.zeros(action_dim, dtype=np.float32).tobytes(),
                'reward': 0.0,
                'done': False,
                'state_ready': False,
                'action_ready': False,
                'exit_flag': False,
                'feedback_ready': False
            })
        
        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
    
    def get_state(self):
        """
        Get the current state from shared memory.
        
        Returns:
            Current state as numpy array
        """
        state_bytes = self.shared_dict.get('state')
        return np.frombuffer(state_bytes, dtype=np.float32)
    
    def set_state(self, state):
        """
        Set the current state in shared memory.
        
        Args:
            state: State to set
        """
        # Convert state to bytes and store in shared memory
        state_array = np.array(state, dtype=np.float32)
        self.shared_dict['state'] = state_array.tobytes()
        self.shared_dict['state_ready'] = True
    
    def get_action(self):
        """
        Get the current action from shared memory.
        
        Returns:
            Current action as numpy array
        """
        action_bytes = self.shared_dict.get('action')
        return np.frombuffer(action_bytes, dtype=np.float32)
    
    def set_action(self, action):
        """
        Set the current action in shared memory.
        
        Args:
            action: Action to set
        """
        # Convert action to bytes and store in shared memory
        action_array = np.array(action, dtype=np.float32)
        self.shared_dict['action'] = action_array.tobytes()
        self.shared_dict['action_ready'] = True
    
    def get_reward(self):
        """
        Get the current reward from shared memory.
        
        Returns:
            Current reward
        """
        return self.shared_dict.get('reward', 0.0)
    
    def set_reward(self, reward):
        """
        Set the current reward in shared memory.
        
        Args:
            reward: Reward to set
        """
        self.shared_dict['reward'] = float(reward)
    
    def get_done(self):
        """
        Get the current done flag from shared memory.
        
        Returns:
            Current done flag
        """
        return self.shared_dict.get('done', False)
    
    def set_done(self, done):
        """
        Set the current done flag in shared memory.
        
        Args:
            done: Done flag to set
        """
        self.shared_dict['done'] = bool(done)
    
    def set_feedback(self, reward, done):
        """
        Set reward and done feedback in shared memory.
        
        Args:
            reward: Reward value
            done: Done flag
        """
        self.set_reward(reward)
        self.set_done(done)
        self.shared_dict['feedback_ready'] = True
    
    def wait_for_state(self, timeout=5.0):
        """
        Wait for state to be ready in shared memory.
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            True if state is ready, False if timeout
        """
        start_time = time.time()
        while not self.shared_dict.get('state_ready', False):
            if time.time() - start_time > timeout:
                logger.warning("Timeout waiting for state")
                return False
            time.sleep(0.001)
        
        self.shared_dict['state_ready'] = False
        return True
    
    def wait_for_action(self, timeout=5.0):
        """
        Wait for action to be ready in shared memory.
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            True if action is ready, False if timeout
        """
        start_time = time.time()
        while not self.shared_dict.get('action_ready', False):
            if time.time() - start_time > timeout:
                logger.warning("Timeout waiting for action")
                return False
            time.sleep(0.001)
        
        self.shared_dict['action_ready'] = False
        return True
    
    def wait_for_feedback(self, timeout=5.0):
        """
        Wait for feedback (reward and done) to be ready in shared memory.
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            True if feedback is ready, False if timeout
        """
        start_time = time.time()
        while not self.shared_dict.get('feedback_ready', False):
            if time.time() - start_time > timeout:
                logger.warning("Timeout waiting for feedback")
                return False
            time.sleep(0.001)
        
        self.shared_dict['feedback_ready'] = False
        return True
    
    def check_exit_flag(self):
        """
        Check if exit flag is set.
        
        Returns:
            True if exit flag is set
        """
        return self.shared_dict.get('exit_flag', False)
    
    def set_exit_flag(self):
        """Set exit flag to signal process termination"""
        self.shared_dict['exit_flag'] = True
    
    def cleanup(self):
        """Clean up shared resources"""
        # Set exit flag to ensure any waiting processes terminate
        self.set_exit_flag()
        
        # Clean up shared memory
        self.shared_dict.cleanup()


class TD3ProcessBridge:
    """
    Process-based implementation of TD3 bridge.
    Creates a separate process for TD3 agent to run in.
    """
    
    def __init__(self, agent_factory, state_dim=24, action_dim=18):
        """
        Initialize the TD3 process bridge.
        
        Args:
            agent_factory: Function that creates TD3 agent
            state_dim: Dimension of the state space
            action_dim: Dimension of the action space
        """
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.agent_factory = agent_factory
        
        # Create shared memory arrays
        self.state_array = mp.Array(ctypes.c_float, state_dim)
        self.action_array = mp.Array(ctypes.c_float, action_dim)
        self.reward_value = mp.Value(ctypes.c_float, 0.0)
        self.done_flag = mp.Value(ctypes.c_bool, False)
        
        # Create synchronization primitives
        self.state_ready = mp.Event()
        self.action_ready = mp.Event()
        self.feedback_ready = mp.Event()
        self.exit_flag = mp.Event()
        
        # Create agent process
        self.process = None
    
    def start(self, agent_args=None):
        """
        Start the TD3 agent process.
        
        Args:
            agent_args: Arguments to pass to agent factory
            
        Returns:
            True if process started successfully
        """
        if self.process is not None and self.process.is_alive():
            logger.warning("TD3 process is already running")
            return False
        
        agent_args = agent_args or {}
        
        # Create and start process
        self.process = mp.Process(
            target=self._agent_process,
            args=(
                self.state_array, self.action_array,
                self.reward_value, self.done_flag,
                self.state_ready, self.action_ready,
                self.feedback_ready, self.exit_flag,
                agent_args
            )
        )
        
        self.process.start()
        logger.info(f"Started TD3 agent process (PID: {self.process.pid})")
        return True
    
    def _agent_process(
        self, state_array, action_array, reward_value, done_flag,
        state_ready, action_ready, feedback_ready, exit_flag, 
        agent_args
    ):
        """
        Main agent process function.
        
        Args:
            state_array: Shared memory for state
            action_array: Shared memory for action
            reward_value: Shared memory for reward
            done_flag: Shared memory for done flag
            state_ready: Event for state ready
            action_ready: Event for action ready
            feedback_ready: Event for feedback ready
            exit_flag: Event for process exit
            agent_args: Arguments for agent factory
        """
        try:
            # Handle signals for clean shutdown
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            
            # Create agent
            logger.info("Creating TD3 agent")
            agent = self.agent_factory(**agent_args)
            
            # Create numpy views of shared arrays
            state = np.frombuffer(state_array.get_obj(), dtype=np.float32)
            action = np.frombuffer(action_array.get_obj(), dtype=np.float32)
            
            logger.info("TD3 agent process ready")
            
            # Main agent loop
            while not exit_flag.is_set():
                # Wait for state
                if state_ready.wait(timeout=1.0):
                    state_ready.clear()
                    
                    # Generate action
                    agent_action = agent.select_action(state)
                    
                    # Copy action to shared memory
                    np.copyto(action, agent_action)
                    
                    # Signal action ready
                    action_ready.set()
                    
                    # Wait for feedback (reward and done)
                    if feedback_ready.wait(timeout=1.0):
                        feedback_ready.clear()
                        
                        # Process feedback if needed
                        # (in online learning scenarios)
                
                # Add a small sleep to prevent 100% CPU usage
                time.sleep(0.001)
                
        except Exception as e:
            logger.exception(f"Error in TD3 agent process: {e}")
        finally:
            logger.info("TD3 agent process exiting")
    
    def set_state(self, state):
        """
        Set state in shared memory and signal state ready.
        
        Args:
            state: State array
        """
        # Copy state to shared memory
        state_view = np.frombuffer(self.state_array.get_obj(), dtype=np.float32)
        np.copyto(state_view, state)
        
        # Signal state ready
        self.state_ready.set()
    
    def get_action(self, timeout=1.0):
        """
        Get action from shared memory.
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            Action array or None if timeout
        """
        # Wait for action ready
        if self.action_ready.wait(timeout=timeout):
            self.action_ready.clear()
            
            # Copy action from shared memory
            action_view = np.frombuffer(self.action_array.get_obj(), dtype=np.float32)
            return action_view.copy()
        
        return None
    
    def set_feedback(self, reward, done):
        """
        Set feedback (reward and done) in shared memory.
        
        Args:
            reward: Reward value
            done: Done flag
        """
        # Set values in shared memory
        self.reward_value.value = reward
        self.done_flag.value = done
        
        # Signal feedback ready
        self.feedback_ready.set()
    
    def stop(self):
        """Stop the TD3 agent process"""
        if self.process is not None:
            # Signal process to exit
            self.exit_flag.set()
            
            # Give process a chance to exit cleanly
            self.process.join(timeout=3.0)
            
            # Force terminate if still alive
            if self.process.is_alive():
                logger.warning("TD3 process did not exit cleanly, terminating")
                self.process.terminate()
                self.process.join(timeout=1.0)
            
            self.process = None
