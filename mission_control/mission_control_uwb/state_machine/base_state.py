#!/usr/bin/env python3
"""Base state class for mission control states."""
from abc import ABC, abstractmethod
from typing import Optional
from rclpy.node import Node

from .states import MissionState
from .mission_context import MissionContext
from ..controller import DroneInterface
from ..aruco import ArucoMarkerHandler
from ..uwb import WaypointManager, UWBNavigator
from ..utils import ParameterLoader


class BaseState(ABC):
    """
    Abstract base class for mission states.
    
    Each concrete state implements the execute() method containing its specific logic.
    """
    
    __slots__ = ('node', 'drone', 'marker_handler', 'waypoint_manager', 'uwb_navigator', 'params', 'context')
    
    def __init__(
        self,
        node: Node,
        drone: DroneInterface,
        marker_handler: ArucoMarkerHandler,
        waypoint_manager: WaypointManager,
        uwb_navigator: UWBNavigator,
        params: ParameterLoader,
        context: MissionContext
    ):
        """
        Initialize base state.
        
        Args:
            node: ROS2 node for logging
            drone: Drone interface for control and telemetry
            marker_handler: ArUco marker coordination handler
            waypoint_manager: Waypoint navigation manager
            uwb_navigator: UWB-based navigator for waypoint navigation
            params: Parameter loader with configuration
            context: Shared mission context for runtime state
        """
        self.node = node
        self.drone = drone
        self.marker_handler = marker_handler
        self.waypoint_manager = waypoint_manager
        self.uwb_navigator = uwb_navigator
        self.params = params
        self.context = context
    
    @abstractmethod
    def execute(self) -> Optional[MissionState]:
        """
        Execute the state logic.
        
        Returns:
            New state to transition to, or None to stay in current state
        """
        pass
