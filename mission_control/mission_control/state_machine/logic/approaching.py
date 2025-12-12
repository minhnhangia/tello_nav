#!/usr/bin/env python3
from typing import Optional
from geometry_msgs.msg import Twist, Pose

from ..states import MissionState
from ..base_state import BaseState


class ApproachingState(BaseState):
    """Handles forward movement toward marker."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute APPROACHING state logic."""
        # Check for marker timeout
        if self.marker_handler.check_marker_timeout(self.params.marker_timeout):
            self.node.get_logger().error(
                f"APPROACHING: Lost marker for >{self.params.marker_timeout}s. "
                "Returning to CENTERING."
            )
            return MissionState.CENTERING
        
        # Check if marker is visible
        if not self.marker_handler.is_marker_visible():
            self._hover_on_temporary_marker_lost()
            return None
        
        self._move_to_marker()

        return None
    
    def _hover_on_temporary_marker_lost(self):
        self.node.get_logger().debug(
            "APPROACHING: Marker temporarily lost. Hovering...",
            throttle_duration_sec=2.0
        )
        self.drone.hover()

    def _move_to_marker(self):
        """Move forward to directly above marker."""
        marker_pose: Pose = self.marker_handler.get_locked_marker_pose()
        if marker_pose is None:
            return
        
        x = marker_pose.position.x
        y = marker_pose.position.y
        z = marker_pose.position.z
        forward_dist = z
        
        self.node.get_logger().info(
            f"APPROACHING: moving forward {forward_dist:.2f} m, "
            f"x = {x:.2f} m, y = {y:.2f} m"
        )
        forward_dist_cmd = self._compute_fwd_dist_cmd(forward_dist)
        self.drone.execute_action(
            f'forward {forward_dist_cmd}',
            MissionState.CAMERA_SWITCHING,
            MissionState.CAMERA_SWITCHING
        )

    def _compute_fwd_dist_cmd(self, forward_dist: float) -> int:
        """Compute forward distance command in cm with final approach offset."""
        return max(0, int((forward_dist - self.params.final_approach_offset) * 100))