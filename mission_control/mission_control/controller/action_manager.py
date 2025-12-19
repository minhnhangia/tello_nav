# action_manager.py

import rclpy
from rclpy.node import Node
from enum import Enum
from typing import TYPE_CHECKING, Callable, Optional

from tello_msgs.msg import TelloResponse
from tello_msgs.srv import TelloAction

if TYPE_CHECKING:
    from ..state_machine.states import MissionState

class ActionState(Enum):
    """Defines the internal states of the ActionManager."""
    IDLE = 0
    WAITING_FOR_ACCEPTANCE = 1
    WAITING_FOR_COMPLETION = 2
    WAITING_FOR_RETRY = 3

class ActionManager:
    """
    A dedicated class to manage the lifecycle of a single Tello action.
    This encapsulates the logic for sending a command, waiting for its
    acceptance, and waiting for its completion, including a timeout.
    
    Supports configurable timeouts and retry logic per command.
    """
    
    # Default parameters
    DEFAULT_TIMEOUT_SEC = 10.0
    DEFAULT_MAX_RETRIES = 0  # No retries by default
    RETRY_DELAY_SEC = 0.5  # Delay between retry attempts
    
    def __init__(
            self, 
            node: Node,
            action_client,
            tello_response_topic: str,
            on_success_callback: Callable,
            on_fail_callback: Callable,
            on_execute_callback: Callable
        ):
        
        self.node = node
        self.logger = self.node.get_logger()
        self.action_client = action_client
        self.state = ActionState.IDLE
        
        # State transition callbacks
        self.on_success_callback = on_success_callback
        self.on_fail_callback = on_fail_callback
        self.on_execute_callback = on_execute_callback
        
        # Current action parameters
        self.pending_next_state = None
        self.pending_fallback_state = None
        self.pending_command: Optional[str] = None
        self.current_timeout = self.DEFAULT_TIMEOUT_SEC
        self.max_retries = self.DEFAULT_MAX_RETRIES
        self.retry_count = 0
        
        # Timing
        self.action_start_time = None
        self.retry_timer = None
        
        # Subscribe to tello_response to know when commands actually complete
        self.tello_response_sub = self.node.create_subscription(
            TelloResponse, tello_response_topic, self.tello_response_callback, 10)

    def is_busy(self):
        """Check if an action is currently executing or pending retry."""
        return self.state != ActionState.IDLE

    def execute_action(
        self, 
        command: str, 
        next_state_on_success: 'MissionState', 
        fallback_state_on_fail: 'MissionState',
        timeout: float = None,
        max_retries: int = None
    ):
        """
        Execute a Tello action command with optional timeout and retry configuration.
        
        Args:
            command: Tello SDK command string (e.g., 'takeoff', 'forward 20')
            next_state_on_success: State to transition to on successful completion
            fallback_state_on_fail: State to transition to after all retries exhausted
            timeout: Command timeout in seconds (default: 10.0)
            max_retries: Maximum retry attempts on failure (default: 0, no retries)
        """
        if self.is_busy():
            self.logger.warning(f"ActionManager is busy, cannot execute '{command}'")
            return

        if not self.action_client.service_is_ready():
            self.logger.error("Cannot execute action: Tello service not available")
            return

        # Store action parameters
        self.pending_command = command
        self.pending_next_state = next_state_on_success
        self.pending_fallback_state = fallback_state_on_fail
        self.current_timeout = timeout if timeout is not None else self.DEFAULT_TIMEOUT_SEC
        self.max_retries = max_retries if max_retries is not None else self.DEFAULT_MAX_RETRIES
        self.retry_count = 0
        
        # Execute the command
        self._send_command()
    
    def _send_command(self):
        """Internal method to send the command to Tello."""
        self.on_execute_callback()
        
        retry_info = f" (attempt {self.retry_count + 1}/{self.max_retries + 1})" if self.max_retries > 0 else ""
        self.logger.info(f'Requesting action: "{self.pending_command}"{retry_info}')

        req = TelloAction.Request()
        req.cmd = self.pending_command
        future = self.action_client.call_async(req)
        future.add_done_callback(self.acceptance_callback)
        self.state = ActionState.WAITING_FOR_ACCEPTANCE

    def acceptance_callback(self, future):
        """Called when TelloAction service responds (command accepted/rejected)."""
        try:
            response = future.result()
            if response.rc == TelloAction.Response.OK:
                self.logger.debug("Command accepted by Tello. Waiting for completion...")
                self.state = ActionState.WAITING_FOR_COMPLETION
                self.action_start_time = self.node.get_clock().now()
            else:
                error_codes = {
                    1: "OK", 2: "ERROR_NOT_CONNECTED", 3: "ERROR_BUSY"
                }
                error_msg = error_codes.get(response.rc, f"Unknown error code: {response.rc}")
                self.logger.error(f"Command rejected with code {response.rc}: {error_msg}")
                
                # Attempt retry if available
                if self.retry_count < self.max_retries:
                    self._schedule_retry(f"rejected with {error_msg}")
                else:
                    self.on_fail_callback(self.pending_fallback_state)
                    self.reset()
                    
        except Exception as e:
            self.logger.error(f'Service call for action acceptance failed: {e}')
            
            # Attempt retry if available
            if self.retry_count < self.max_retries:
                self._schedule_retry(f"service exception: {e}")
            else:
                self.on_fail_callback(self.pending_fallback_state)
                self.reset()

    def tello_response_callback(self, msg: TelloResponse):
        """Called when a Tello command actually completes (not just accepted)."""
        if self.state != ActionState.WAITING_FOR_COMPLETION:
            return # Ignore unexpected responses

        if msg.rc == TelloResponse.OK and msg.str == "ok":
            self.logger.debug(f"Command completed successfully: '{msg.str}'")
            self.on_success_callback(self.pending_next_state)
            self.reset()
        else:
            self.logger.error(f"Command failed during execution: '{msg.str}'")
            
            # Attempt retry if available
            if self.retry_count < self.max_retries:
                self._schedule_retry(f"execution failed: {msg.str}")
            else:
                self.on_fail_callback(self.pending_fallback_state)
                self.reset()

    def check_timeout(self):
        """Called periodically by the main node's timer to check for action timeouts."""
        if self.state == ActionState.WAITING_FOR_COMPLETION and self.action_start_time is not None:
            elapsed = (self.node.get_clock().now() - self.action_start_time).nanoseconds / 1e9
            if elapsed > self.current_timeout:
                self.logger.error(f"Action timed out after {elapsed:.1f}s.")
                
                # Attempt retry if available
                if self.retry_count < self.max_retries:
                    self._schedule_retry(f"timeout after {elapsed:.1f}s")
                else:
                    self.on_fail_callback(self.pending_fallback_state)
                    self.reset()
    
    def _schedule_retry(self, reason: str):
        """Schedule a retry attempt after a brief delay."""
        self.retry_count += 1
        self.logger.warning(
            f"Retrying command '{self.pending_command}' "
            f"(attempt {self.retry_count + 1}/{self.max_retries + 1}). "
            f"Reason: {reason}"
        )
        
        self.state = ActionState.WAITING_FOR_RETRY
        
        # Cancel any existing retry timer
        if self.retry_timer is not None:
            self.retry_timer.cancel()
        
        # Schedule retry after delay
        self.retry_timer = self.node.create_timer(
            self.RETRY_DELAY_SEC,
            self._execute_retry
        )
    
    def _execute_retry(self):
        """Execute the retry attempt (called by timer)."""
        # Cancel and cleanup timer
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None
        
        # Send the command again
        self._send_command()

    def reset(self):
        """Resets the manager to its initial state."""
        # Cancel any pending retry timer
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None
        
        self.state = ActionState.IDLE
        self.pending_next_state = None
        self.pending_fallback_state = None
        self.pending_command = None
        self.action_start_time = None
        self.retry_count = 0
        self.max_retries = self.DEFAULT_MAX_RETRIES
        self.current_timeout = self.DEFAULT_TIMEOUT_SEC
