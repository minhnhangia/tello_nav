#!/usr/bin/env python3
from typing import Optional
from geometry_msgs.msg import Pose

from ..states import MissionState
from ..base_state import BaseState


class WaypointApproachingState(BaseState):
    """Handles forward movement toward waypoint marker with downward camera.
    
    Flow:
    1. Switch to downward camera
    2. Move forward continuously 
    3. When downward camera detects ArUco, switch back to forward camera
    4. Transition to WAYPOINT_ACTION
    """
    
    def execute(self) -> Optional[MissionState]:
        """Execute WAYPOINT_APPROACHING state logic with downward camera marker detection."""
        # Check if waypoint sequence is complete
        if self.waypoint_manager.is_sequence_complete():  # type: ignore
            self.node.get_logger().error(
                "WAYPOINT_APPROACHING: Waypoint sequence complete. "
                "Transitioning to SEARCHING."
            )
            return MissionState.SEARCHING
        
        # Defensive check: action-only waypoints should not reach this state
        if self.waypoint_manager.is_current_waypoint_action_only():  # type: ignore
            self.node.get_logger().warning(
                "WAYPOINT_APPROACHING: Action-only waypoint detected. "
                "Transitioning to WAYPOINT_ACTION."
            )
            return MissionState.WAYPOINT_ACTION

        # ====================================================================
        # PHASE 1: SWITCH TO DOWNWARD CAMERA
        # ====================================================================
        if not self.context.camera_switched:
            self.node.get_logger().info(
                "WAYPOINT_APPROACHING: Switching to downward camera and starting forward movement."
            )
            self.drone.execute_action(
                'downvision 1',
                MissionState.WAYPOINT_APPROACHING,  # Stay in state after switch
                MissionState.SEARCHING,
                max_retries=5
            )
            self.context.camera_switched = True
            self.context.downward_camera_active = True
            self.waypoint_manager.clear_marker_pose()
            # START MOVING FORWARD IMMEDIATELY
            self._move_forward_with_downward_camera()
            return None  # Continue to next tick to check for marker
        
        # ====================================================================
        # PHASE 2: CHECK IF DOWNWARD CAMERA DETECTED MARKER
        # ====================================================================
        # Check if marker was detected with downward camera
        if self.context.downward_camera_active:
            self._move_forward_with_downward_camera()
            
            if self.waypoint_manager.is_marker_visible_on_downward_camera():  # type: ignore
                self.node.get_logger().info(
                    "WAYPOINT_APPROACHING: Downward camera detected marker! Switching back to forward camera."
                )
                self.context.marker_detected_with_down_camera = True
                self.drone.hover()  # Stop moving while switching cameras
                # Switch back to forward camera
                self.drone.execute_action(
                    'downvision 0',
                    MissionState.WAYPOINT_APPROACHING,  # Stay in state after switch
                    MissionState.SEARCHING,
                    max_retries=5
                )
                self.context.downward_camera_active = False
                return None  # Wait for camera switch to complete
            
            return None
        
        # ====================================================================
        # PHASE 3: IF FORWARD CAMERA IS ACTIVE, TRANSITION TO NEXT STATE
        # ====================================================================
        if self.context.marker_detected_with_down_camera and not self.context.downward_camera_active:
            self.node.get_logger().info(
                "WAYPOINT_APPROACHING: Forward camera active. Transitioning to WAYPOINT_ACTION."
            )
            return MissionState.WAYPOINT_ACTION
        
    def _hover_on_temporary_marker_lost(self):
        self.node.get_logger().debug(
            "WAYPOINT_APPROACHING: Marker temporarily lost. Hovering...",
            throttle_duration_sec=2.0
        )
        self.drone.hover()
    
    def _move_forward_with_downward_camera(self):
        """Publish continuous forward velocity while downward camera is active."""
        forward_speed = self.params.centering_forward_speed
        self.drone.move_forward(forward_speed)
        
        self.node.get_logger().debug(
            "WAYPOINT_APPROACHING: Moving forward with downward camera active (waiting for marker detection)...",
            throttle_duration_sec=2.0
        )