#!/usr/bin/env python3
"""
Waypoint Manager Module

Manages sequential waypoint navigation for autonomous missions.
"""

from dataclasses import dataclass
from typing import Optional, List
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose
from aruco_opencv_msgs.msg import ArucoDetection


@dataclass
class WaypointConfig:
    """
    Configuration for a single waypoint.
    
    Attributes:
        marker_id: ArUco marker ID to navigate to
        post_action: Optional Tello SDK command to execute after reaching waypoint
                     (e.g., "ccw 90", "cw 45", None)
    """
    marker_id: int
    post_action: Optional[str] = None


class WaypointManager:
    """
    Manages sequential waypoint navigation.
    
    Responsibilities:
    - Store waypoint sequence from parameters
    - Track current waypoint index
    - Provide current target marker ID
    - Store/update current waypoint marker pose from ArUco
    - Track marker visibility and timeout
    - Advance through waypoint sequence
    """
    
    def __init__(self, node: Node, waypoint_sequence: List[str]):
        """
        Initialize waypoint manager.
        
        Args:
            node: ROS2 node for logging and time
            waypoint_sequence: List of waypoint strings from parameters
                              Format: ["id:50,action:ccw 90", "id:51,action:none", ...]
        """
        self.node = node
        self.logger = node.get_logger()
        
        # Parse waypoint sequence
        self.waypoints: List[WaypointConfig] = self._parse_waypoint_sequence(waypoint_sequence)
        
        # Current state
        self.current_index = 0
        self.marker_pose: Optional[Pose] = None
        self.marker_last_seen_time: Optional[Time] = None
        
        if self.waypoints:
            self.logger.info(
                f"WaypointManager initialized with {len(self.waypoints)} waypoints: "
                f"{[w.marker_id for w in self.waypoints]}"
            )
        else:
            self.logger.warning("WaypointManager initialized with empty waypoint sequence")
    
    def _parse_waypoint_sequence(self, waypoint_sequence: List[str]) -> List[WaypointConfig]:
        """
        Parse waypoint configuration from string array.
        
        Args:
            waypoint_sequence: List of waypoint strings
                             Format: ["id:50,action:ccw 90", "id:51,action:none", ...]
            
        Returns:
            List of WaypointConfig objects
        """
        waypoints = []
        for wp_str in waypoint_sequence:
            try:
                # Parse "id:50,action:ccw 90" format
                marker_id = None
                post_action = None
                
                parts = wp_str.split(',')
                for part in parts:
                    key, value = part.split(':', 1)
                    key = key.strip()
                    value = value.strip()
                    
                    if key == 'id':
                        marker_id = int(value)
                    elif key == 'action':
                        # Treat "none", "null", empty string as None
                        post_action = value if value.lower() not in ['none', 'null', ''] else None
                
                if marker_id is None:
                    self.logger.error(f"Waypoint missing 'id' field: {wp_str}")
                    continue
                
                waypoints.append(WaypointConfig(
                    marker_id=marker_id,
                    post_action=post_action
                ))
            except Exception as e:
                self.logger.error(f"Failed to parse waypoint '{wp_str}': {e}")
                continue
        
        return waypoints
    
    def get_current_waypoint(self) -> Optional[WaypointConfig]:
        """
        Get the current active waypoint configuration.
        
        Returns:
            Current WaypointConfig or None if sequence complete
        """
        if 0 <= self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        return None
    
    def get_current_marker_id(self) -> int:
        """
        Get the marker ID of the current waypoint.
        
        Returns:
            Current waypoint marker ID or -1 if no waypoint active
        """
        waypoint = self.get_current_waypoint()
        return waypoint.marker_id if waypoint else -1
    
    def update_marker_pose(self, msg: ArucoDetection):
        """
        Update marker pose from ArUco detection message.
        
        Searches for the current waypoint marker in the detection and updates pose.
        
        Args:
            msg: ArucoDetection message containing detected markers
        """
        current_marker_id = self.get_current_marker_id()
        if current_marker_id < 0:
            return
        
        # Search for current waypoint marker
        marker_found = False
        for marker in msg.markers:
            if marker.marker_id == current_marker_id:
                self.marker_pose = marker.pose
                self.marker_last_seen_time = self.node.get_clock().now()
                marker_found = True
                break
        
        # Clear pose if marker not visible
        if not marker_found:
            self.marker_pose = None
    
    def get_marker_pose(self) -> Optional[Pose]:
        """
        Get the latest pose of the current waypoint marker.
        
        Returns:
            Marker pose or None if not currently visible
        """
        return self.marker_pose
    
    def is_marker_visible(self) -> bool:
        """
        Check if the current waypoint marker is currently visible.
        
        Returns:
            True if marker pose is available, False otherwise
        """
        return self.marker_pose is not None
    
    def check_marker_timeout(self, timeout_duration: float) -> bool:
        """
        Check if the current waypoint marker has timed out.
        
        Args:
            timeout_duration: Timeout threshold in seconds
            
        Returns:
            True if marker has been lost longer than timeout, False otherwise
        """
        if self.marker_last_seen_time is None:
            return False
        
        time_since_last_seen = (
            self.node.get_clock().now() - self.marker_last_seen_time
        ).nanoseconds / 1e9
        
        return time_since_last_seen > timeout_duration
    
    def advance_to_next(self) -> bool:
        """
        Advance to the next waypoint in the sequence.
        
        Returns:
            True if there are more waypoints, False if sequence complete
        """
        self.current_index += 1
        
        # Clear marker state for new waypoint
        self.marker_pose = None
        self.marker_last_seen_time = None
        
        if self.current_index < len(self.waypoints):
            next_waypoint = self.waypoints[self.current_index]
            self.logger.info(
                f"Advanced to waypoint {self.current_index + 1}/{len(self.waypoints)}: "
                f"Marker ID {next_waypoint.marker_id}"
            )
            return True
        else:
            self.logger.info("All waypoints completed!")
            return False
    
    def has_next_waypoint(self) -> bool:
        """
        Check if there are more waypoints after the current one.
        
        Returns:
            True if more waypoints exist, False otherwise
        """
        return (self.current_index + 1) < len(self.waypoints)
    
    def is_sequence_complete(self) -> bool:
        """
        Check if all waypoints have been completed.
        
        Returns:
            True if all waypoints done, False otherwise
        """
        return self.current_index >= len(self.waypoints)
    
    def reset(self):
        """
        Reset waypoint manager to initial state.
        
        Resets to first waypoint and clears all marker tracking.
        """
        self.current_index = 0
        self.marker_pose = None
        self.marker_last_seen_time = None
        self.logger.info("WaypointManager reset to initial state")
    
    def get_waypoint_count(self) -> int:
        """
        Get total number of waypoints in sequence.
        
        Returns:
            Total waypoint count
        """
        return len(self.waypoints)
    
    def get_current_index(self) -> int:
        """
        Get current waypoint index (0-based).
        
        Returns:
            Current index
        """
        return self.current_index