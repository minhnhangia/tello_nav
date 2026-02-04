#!/usr/bin/env python3
"""
Waypoint Manager Module

Manages sequential UWB coordinate-based waypoint navigation for autonomous missions.
Waypoints are loaded from a JSON file for clean separation of concerns.
"""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, List, Tuple

from rclpy.node import Node


@dataclass(frozen=True, slots=True)
class Waypoint:
    """
    Immutable configuration for a single waypoint.
    
    Attributes:
        x: X coordinate in meters (world frame)
        y: Y coordinate in meters (world frame)
        label: Optional human-readable label for logging
    """
    x: float
    y: float
    label: Optional[str] = None
    
    @property
    def position(self) -> Tuple[float, float]:
        """Return (x, y) tuple for convenience."""
        return (self.x, self.y)
    
    def __str__(self) -> str:
        if self.label:
            return f"{self.label}({self.x:.2f}, {self.y:.2f})"
        return f"({self.x:.2f}, {self.y:.2f})"


class WaypointManager:
    """
    Manages sequential waypoint navigation using UWB coordinates.
    
    Waypoints are loaded from a JSON file with format:
        {
          "waypoints": [
            {"x": 1.0, "y": 2.0, "label": "optional_name"},
            {"x": 3.0, "y": 4.0}
          ]
        }
    
    Responsibilities:
        - Load waypoints from JSON file
        - Track current waypoint index
        - Provide current target coordinates
        - Advance through waypoint sequence (including skip on timeout)
    """
    
    __slots__ = ('_node', '_waypoints', '_current_index')
    
    def __init__(self, node: Node, waypoints_file: str):
        """
        Initialize waypoint manager from JSON file.
        
        Args:
            node: ROS2 node for logging
            waypoints_file: Path to JSON waypoints file
            
        Raises:
            FileNotFoundError: If waypoints file doesn't exist
            ValueError: If JSON format is invalid
        """
        self._node = node
        self._waypoints: Tuple[Waypoint, ...] = self._load_waypoints(waypoints_file)
        self._current_index: int = 0
        
        self._log_initialization()
    
    def _load_waypoints(self, filepath: str) -> Tuple[Waypoint, ...]:
        """
        Load waypoints from JSON file.
        
        Args:
            filepath: Path to JSON waypoints file
            
        Returns:
            Tuple of Waypoint objects
            
        Raises:
            FileNotFoundError: If file doesn't exist
            ValueError: If JSON format is invalid
        """
        path = Path(filepath).expanduser().resolve()
        logger = self._node.get_logger()
        
        if not path.exists():
            raise FileNotFoundError(f"Waypoints file not found: {path}")
        
        logger.info(f"WaypointManager: Loading waypoints from {path}")
        
        with open(path, 'r') as f:
            data = json.load(f)
        
        # Validate structure
        if 'waypoints' not in data:
            raise ValueError(
                f"Invalid waypoints file: missing 'waypoints' key. "
                f"Expected format: {{'waypoints': [{{'x': 1.0, 'y': 2.0}}, ...]}}"
            )
        
        waypoints_data = data['waypoints']
        if not isinstance(waypoints_data, list):
            raise ValueError("'waypoints' must be an array")
        
        # Parse waypoints
        waypoints: List[Waypoint] = []
        for i, wp in enumerate(waypoints_data):
            if not isinstance(wp, dict):
                raise ValueError(f"Waypoint {i} must be an object with 'x' and 'y' keys")
            
            if 'x' not in wp or 'y' not in wp:
                raise ValueError(f"Waypoint {i} missing required 'x' or 'y' coordinate")
            
            waypoints.append(Waypoint(
                x=float(wp['x']),
                y=float(wp['y']),
                label=wp.get('label')
            ))
        
        return tuple(waypoints)
    
    def _log_initialization(self) -> None:
        """Log waypoint initialization summary."""
        logger = self._node.get_logger()
        
        if not self._waypoints:
            logger.warning("WaypointManager: No waypoints in file")
            return
        
        waypoint_strs = [str(wp) for wp in self._waypoints]
        logger.info(
            f"WaypointManager: Loaded {len(self._waypoints)} waypoints: "
            f"{', '.join(waypoint_strs)}"
        )
    
    # ========================================================================
    # WAYPOINT ACCESS
    # ========================================================================
    
    def get_current_waypoint(self) -> Optional[Waypoint]:
        """
        Get the current active waypoint.
        
        Returns:
            Current Waypoint or None if sequence complete/empty
        """
        if 0 <= self._current_index < len(self._waypoints):
            return self._waypoints[self._current_index]
        return None
    
    def get_current_position(self) -> Optional[Tuple[float, float]]:
        """
        Get (x, y) coordinates of current waypoint.
        
        Convenience method for direct coordinate access.
        
        Returns:
            (x, y) tuple in meters, or None if no active waypoint
        """
        waypoint = self.get_current_waypoint()
        return waypoint.position if waypoint else None
    
    # ========================================================================
    # SEQUENCE CONTROL
    # ========================================================================
    
    def advance(self) -> bool:
        """
        Advance to the next waypoint in the sequence.
        
        Call this when current waypoint is reached OR on timeout (skip).
        
        Returns:
            True if advanced to a new waypoint, False if sequence complete
        """
        if self._current_index >= len(self._waypoints):
            return False
        
        self._current_index += 1
        
        if self._current_index < len(self._waypoints):
            next_wp = self._waypoints[self._current_index]
            self._node.get_logger().info(
                f"WaypointManager: Advanced to waypoint "
                f"{self.get_progress_string()}: {next_wp}"
            )
            return True
        
        self._node.get_logger().info(
            "WaypointManager: All waypoints completed!"
        )
        return False
    
    def skip_current(self) -> bool:
        """
        Skip the current waypoint (e.g., on timeout) and advance to next.
        
        Logs a warning before advancing.
        
        Returns:
            True if skipped to a new waypoint, False if sequence complete
        """
        current = self.get_current_waypoint()
        if current:
            self._node.get_logger().warning(
                f"WaypointManager: Skipping waypoint "
                f"{self.get_progress_string()}: {current}"
            )
        return self.advance()
    
    def reset(self) -> None:
        """Reset to the first waypoint."""
        self._current_index = 0
        self._node.get_logger().info("WaypointManager: Reset to first waypoint")
    
    # ========================================================================
    # SEQUENCE INTROSPECTION
    # ========================================================================
    
    def is_complete(self) -> bool:
        """Check if all waypoints have been visited."""
        return self._current_index >= len(self._waypoints)
    
    def has_waypoints(self) -> bool:
        """Check if any waypoints are configured."""
        return len(self._waypoints) > 0
    
    @property
    def current_index(self) -> int:
        """Current waypoint index (0-based)."""
        return self._current_index
    
    @property
    def total_count(self) -> int:
        """Total number of waypoints in sequence."""
        return len(self._waypoints)
    
    def get_progress_string(self) -> str:
        """
        Get human-readable progress string.
        
        Returns:
            String like "2/5" (1-indexed for display)
        """
        return f"{self._current_index + 1}/{len(self._waypoints)}"
    
    @property
    def waypoints(self) -> Tuple[Waypoint, ...]:
        """Read-only access to all waypoints."""
        return self._waypoints