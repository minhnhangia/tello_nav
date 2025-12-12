# action_manager.py

import rclpy
from rclpy.node import Node
from enum import Enum
from typing import TYPE_CHECKING, Callable

from tello_msgs.msg import TelloResponse
from tello_msgs.srv import TelloAction

if TYPE_CHECKING:
    from ..state_machine.states import MissionState

class ActionState(Enum):
    """Defines the internal states of the ActionManager."""
    IDLE = 0
    WAITING_FOR_ACCEPTANCE = 1
    WAITING_FOR_COMPLETION = 2

class ActionManager:
    """
    A dedicated class to manage the lifecycle of a single Tello action.
    This encapsulates the logic for sending a command, waiting for its
    acceptance, and waiting for its completion, including a timeout.
    """
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
        self.pending_next_state = None
        self.pending_fallback_state = None
        self.action_start_time = None
        self.ACTION_TIMEOUT_SEC = 10.0
        # Callbacks to notify the main MissionControl node
        self.on_success_callback = on_success_callback
        self.on_fail_callback = on_fail_callback
        self.on_execute_callback = on_execute_callback
        
        # Subscribe to tello_response to know when commands actually complete
        self.tello_response_sub = self.node.create_subscription(
            TelloResponse, tello_response_topic, self.tello_response_callback, 10)

    def is_busy(self):
        return self.state != ActionState.IDLE

    def execute_action(self, command: str, next_state_on_success: 'MissionState', fallback_state_on_fail: 'MissionState'):
        if self.is_busy():
            self.logger.warning(f"ActionManager is busy, cannot execute '{command}'")
            return

        if not self.action_client.service_is_ready():
            self.logger.error("Cannot execute action: Tello service not available")
            return

        self.on_execute_callback()
        self.pending_next_state = next_state_on_success
        self.pending_fallback_state = fallback_state_on_fail
        self.logger.info(f'Requesting action: "{command}"')

        req = TelloAction.Request()
        req.cmd = command
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
                self.on_fail_callback(self.pending_fallback_state) # Notify main node of failure
                self.reset()
        except Exception as e:
            self.logger.error(f'Service call for action acceptance failed: {e}')
            self.on_fail_callback(self.pending_fallback_state)
            self.reset()

    def tello_response_callback(self, msg: TelloResponse):
        """Called when a Tello command actually completes (not just accepted)."""
        if self.state != ActionState.WAITING_FOR_COMPLETION:
            return # Ignore unexpected responses

        if msg.rc == TelloResponse.OK and msg.str == "ok":
            self.logger.debug(f"Command completed successfully: '{msg.str}'")
            self.on_success_callback(self.pending_next_state) # Notify main node
        else:
            self.logger.error(f"Command failed during execution: '{msg.str}'")
            self.on_fail_callback(self.pending_fallback_state)

        self.reset()

    def check_timeout(self):
        """Called periodically by the main node's timer."""
        if self.state == ActionState.WAITING_FOR_COMPLETION and self.action_start_time is not None:
            elapsed = (self.node.get_clock().now() - self.action_start_time).nanoseconds / 1e9
            if elapsed > self.ACTION_TIMEOUT_SEC:
                self.logger.error(f"Action timed out after {elapsed:.1f}s. Failing.")
                self.on_fail_callback(self.pending_fallback_state)
                self.reset()

    def reset(self):
        """Resets the manager to its initial state."""
        self.state = ActionState.IDLE
        self.pending_next_state = None
        self.pending_fallback_state = None
        self.action_start_time = None