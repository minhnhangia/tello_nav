#!/usr/bin/env python3
"""Mission manager coordinating state execution."""
from typing import Optional, Dict
from rclpy.node import Node

from .states import MissionState
from .base_state import BaseState
from .mission_context import MissionContext
from .state_implementations import (
    TakingOffState,
    AscendingState,
    SearchingState,
    CenteringState,
    ApproachingState,
    CameraSwitchingState,
    PrecisionLandingState,
    LandingState,
    CompletingMissionState,
    ResettingState
)
from ..controller import DroneInterface
from ..aruco import ArucoMarkerHandler
from ..utils import ParameterLoader

StateRegistry = Dict[MissionState, BaseState]


class MissionManager:
    """
    Manages mission state coordination and execution.
    
    Uses the State Pattern with Context Object Pattern:
    - Each mission state is encapsulated in its own class with an execute() method
    - Shared runtime state lives in MissionContext, avoiding inter-state coupling
    - Manager coordinates state transitions and maintains state instances
    """
    
    def __init__(
        self,
        node: Node,
        drone: DroneInterface,
        marker_handler: ArucoMarkerHandler,
        params: ParameterLoader
    ):
        """
        Initialize mission manager with state pattern architecture.
        
        Args:
            node: ROS2 node for logging
            drone: Drone interface for control and telemetry
            marker_handler: ArUco marker coordination handler
            params: Parameter loader with configuration
        """
        self.node = node
        self.drone = drone
        self.marker_handler = marker_handler
        self.params = params
        
        # Shared mission context
        self.context = MissionContext()
        
        # Initialize all state instances
        self._states: StateRegistry = self._create_states()
    
    def _create_states(self) -> StateRegistry:
        """
        Create instances of all state classes.
        
        Returns:
            Dictionary mapping MissionState enum to state instance
        """
        common_args = (self.node, self.drone, self.marker_handler, self.params, self.context)
        
        return {
            MissionState.TAKING_OFF: TakingOffState(*common_args),
            MissionState.ASCENDING: AscendingState(*common_args),
            MissionState.SEARCHING: SearchingState(*common_args),
            MissionState.CENTERING: CenteringState(*common_args),
            MissionState.APPROACHING: ApproachingState(*common_args),
            MissionState.CAMERA_SWITCHING: CameraSwitchingState(*common_args),
            MissionState.PRECISION_LANDING: PrecisionLandingState(*common_args),
            MissionState.LANDING: LandingState(*common_args),
            MissionState.COMPLETING_MISSION: CompletingMissionState(*common_args),
            MissionState.RESETTING: ResettingState(*common_args),
        }
    
    def execute_state(self, state: MissionState) -> Optional[MissionState]:
        """
        Execute logic for the given state.
        
        Args:
            state: Current mission state to execute
            
        Returns:
            New state to transition to, or None to stay in current state
        """
        state_instance = self._states.get(state)
        if state_instance:
            new_state = state_instance.execute()
            if new_state is not None and new_state != state:
                self.node.get_logger().debug(
                    f"State transition: {state.name} → {new_state.name}"
                )
            return new_state
        
        self.node.get_logger().error(f"Unknown state: {state}")
        return None
    
    # ========================================================================
    # PUBLIC INTERFACE FOR EXTERNAL ACCESS TO CONTEXT VARIABLES
    # ========================================================================
    
    @property
    def is_near_exit(self) -> bool:
        """Get exit marker proximity flag from mission context."""
        return self.context.is_near_exit
    
    @is_near_exit.setter
    def is_near_exit(self, value: bool):
        """Set exit marker proximity flag in mission context."""
        self.context.is_near_exit = value
    
    @property
    def is_blind_landing(self) -> bool:
        """Get blind landing flag from mission context."""
        return self.context.is_blind_landing
    
    @is_blind_landing.setter
    def is_blind_landing(self, value: bool):
        """Set blind landing flag in mission context."""
        self.context.is_blind_landing = value
