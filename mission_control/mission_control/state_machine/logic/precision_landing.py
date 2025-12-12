#!/usr/bin/env python3
import numpy as np
from typing import Optional
from geometry_msgs.msg import Twist, Pose

from ..states import MissionState
from ..base_state import BaseState


class PrecisionLandingState(BaseState):
    """Handles fine-tuned positioning over marker with recovery maneuvers."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute PRECISION_LANDING state logic with recovery maneuvers."""
        # Check for full timeout - land blind
        if self.marker_handler.check_marker_timeout(self.params.precision_landing_timeout):
            self._handle_permanent_marker_lost()
            return MissionState.LANDING
        
        # Check for partial timeout
        if self.marker_handler.check_marker_timeout(self.params.precision_landing_timeout / 2):
            if self.drone.latest_height < self.params.recovery_height:
                self._move_up_to_find_marker()
                return None
        
        # Check if marker is visible
        if not self.marker_handler.is_marker_visible():
            self._hover_on_temporary_marker_lost()
            return None
        
        # Extract positional errors
        marker_pose: Pose = self.marker_handler.get_locked_marker_pose()
        y_error = marker_pose.position.y
        x_error = marker_pose.position.x
        
        # Check if centered on both axes
        x_centered = abs(x_error) <= self.params.precision_landing_threshold_x
        y_centered = abs(y_error) <= self.params.precision_landing_threshold_y
        
        if x_centered and y_centered:
            self._handle_successful_centering()
            return MissionState.LANDING
        
        self._perform_precision_alignment(x_centered, x_error, y_centered, y_error)

        return None
    
    def _handle_permanent_marker_lost(self):
        self.node.get_logger().warning(
            f"PRECISION_LANDING: Lost marker for >{self.params.precision_landing_timeout}s. "
            "Landing blind!"
        )
        self.drone.hover()
        self.context.is_blind_landing = True

    def _move_up_to_find_marker(self):
        self.node.get_logger().warning(
            f"PRECISION_LANDING: Lost marker for "
            f">{self.params.precision_landing_timeout/2}s. Moving up!",
            throttle_duration_sec=2.0
        )
        self.drone.move_up(self.params.ascending_speed)

    def _hover_on_temporary_marker_lost(self):
        self.node.get_logger().debug(
            "PRECISION_LANDING: Marker temporarily lost. Hovering...",
            throttle_duration_sec=2.0
        )
        self.drone.hover()

    def _handle_successful_centering(self):
        self.node.get_logger().info(
            "PRECISION_LANDING: Centered over marker. Transitioning to LANDING."
        )
        self.drone.hover()
        self.context.is_blind_landing = False

    def _perform_precision_alignment(self, x_centered: bool, x_error: float,
                                     y_centered: bool, y_error: float):
        """Perform precision alignment over marker using proportional control."""
        twist_msg = Twist()
        if not x_centered:
            twist_msg.linear.x = self._compute_speed(-x_error,
                                              self.params.precision_forward_kp,
                                              self.params.precision_landing_max_speed)
        
        if not y_centered:
            twist_msg.linear.y = self._compute_speed(-y_error,
                                              self.params.precision_sideway_kp,
                                              self.params.precision_landing_max_speed)
        
        self.node.get_logger().debug (
            f"PRECISION_LANDING: x_err: {x_error:.2f}, y_err: {y_error:.2f}, "
            f"speed: {twist_msg.linear}",
            throttle_duration_sec=1.0
        )
        self.drone.publish_velocity(twist_msg)

    def _compute_speed(self, error: float, kp: float, max_speed: float) -> float:
        """Compute velocity command using proportional control."""
        return np.clip(
            error * kp,
            -max_speed,
            max_speed
        )