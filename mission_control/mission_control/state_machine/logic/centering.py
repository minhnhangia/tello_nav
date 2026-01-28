#!/usr/bin/env python3
import numpy as np
from typing import Optional
from geometry_msgs.msg import Twist, Pose
from midas_msgs.msg import DepthMapAnalysis

from ..states import MissionState
from ..base_state import BaseState


class CenteringState(BaseState):
    """Handles yaw alignment on locked marker with obstacle avoidance."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute CENTERING state logic with obstacle avoidance and yaw alignment."""
        # Check for marker timeout
        if self.marker_handler.check_marker_timeout(self.params.marker_timeout):
            self._handle_permanent_marker_lost()
            return MissionState.SEARCHING
        
        if not self.drone.has_sensor_data():
            self._wait_for_sensor_data()
            return None
        # Get sensor data
        depth: DepthMapAnalysis = self.drone.latest_depth_analysis # type: ignore
        tof: float = self.drone.latest_tof # type: ignore
        
        # Check if marker is visible
        if not self.marker_handler.is_marker_visible():
            self._handle_temporary_marker_lost(depth, tof)
            return None
        marker_pose: Pose = self.marker_handler.get_locked_marker_pose() # type: ignore
        x_error = marker_pose.position.x
        forward_dist = marker_pose.position.z
        is_centered = abs(x_error) < self.params.centering_threshold_x
        is_close_enough = forward_dist < self.params.max_approach_dist 
        
        # --- 1. OBSTACLE AVOIDANCE ---
        if self._avoid_obstacle_if_needed(depth, x_error):
            return None
        
        # --- 2. YAW CENTERING ---
        if not is_centered:
            self._perform_yaw_centering(x_error)
            return None

        # --- 3. DECIDE TO APPROACH ---
        # Check if centered and close enough to approach
        if (is_centered and is_close_enough):
            self._handle_successful_centering()
            return MissionState.APPROACHING
        
        # Centered but far - move closer
        if (is_centered and not is_close_enough):
            self.drone.move_forward(self.params.forward_speed * 2)
            return None
        
    def _wait_for_sensor_data(self):
        self.node.get_logger().debug(
            'CENTERING: Waiting for sensor data...',
            throttle_duration_sec=5
        )
        self.drone.hover()

    def _handle_permanent_marker_lost(self):
        self.node.get_logger().warning(
            f"CENTERING: Lost marker for >{self.params.marker_timeout}s. "
            "Returning to SEARCHING."
        )
        self.marker_handler.unreserve_current_marker()

    def _handle_temporary_marker_lost(self, depth: DepthMapAnalysis, tof: float):
        if not self.drone.has_sensor_data():
            return
        
        is_obstacle_detected = depth.middle_center.red > depth.middle_center.blue

        is_corner_detected = (depth.middle_center.blue > depth.middle_center.red and 
                              tof <= self.params.corner_tof_threshold)
        
        is_headon_detected = tof <= self.params.headon_tof_threshold

        is_safe_to_move_forward = not (is_obstacle_detected or 
                                       is_corner_detected or 
                                       is_headon_detected)

        # Check if already close enough based on last known position
        last_pose = self.marker_handler.get_last_known_marker_pose()
        is_close_enough = (last_pose is not None and 
                           last_pose.position.z < self.params.max_approach_dist)

        if is_safe_to_move_forward and not is_close_enough:
            self.node.get_logger().debug(
                "CENTERING: Marker temporarily lost. Moving forward to find...",
                throttle_duration_sec=2.0
            )
            self.drone.move_forward(self.params.centering_forward_speed)
        else:
            self._hover_on_temporary_marker_lost()

    def _hover_on_temporary_marker_lost(self):
        self.node.get_logger().debug(
            "CENTERING: Marker temporarily lost. Hovering...",
            throttle_duration_sec=2.0
        )
        self.drone.hover()

    def _perform_yaw_centering(self, x_error: float):
        twist_msg = Twist()
        twist_msg.angular.z = self._compute_speed(
            error=x_error,
            kp=self.params.centering_yaw_kp,
            max_speed=self.params.centering_yaw_speed
        )
        self.node.get_logger().debug(
            f"CENTERING: x_error: {x_error:.2f}m, "
            f"yaw_speed: {twist_msg.angular.z:.2f}",
            throttle_duration_sec=1.0
        )
        self.drone.publish_velocity(twist_msg)

    def _avoid_obstacle_if_needed(self, depth: DepthMapAnalysis, 
                                  x_error: float) -> bool:
        is_obstacle_left = depth.mc_left.nonblue - 100 > depth.mc_left.blue
        is_obstacle_right = depth.mc_right.nonblue - 100 > depth.mc_right.blue
        is_marker_centered = abs(x_error) < self.params.centering_threshold_x
        is_marker_left = x_error <= -self.params.centering_threshold_x
        is_marker_right = x_error >= self.params.centering_threshold_x

        if is_obstacle_left and not is_marker_left:
            self.node.get_logger().warning(
                "CENTERING: Obstacle on the left, "
                "marker on the right, "
                "moving RIGHT.",
                throttle_duration_sec=1.0
            )
            self.drone.move_right(self.params.sideway_speed)
            return True
        
        elif is_obstacle_right and not is_marker_right:
            self.node.get_logger().warning(
                "CENTERING: Obstacle on the right, "
                "marker on the left, "
                "moving LEFT.",
                throttle_duration_sec=1.0
            )
            self.drone.move_left(self.params.sideway_speed)
            return True
        
        return False

    def _handle_successful_centering(self):
        self.node.get_logger().info(
            "CENTERING: Centered on marker. Transitioning to APPROACHING."
        )
        self.drone.hover()

    def _compute_speed(self, error: float, kp: float, max_speed: float) -> float:
        """Compute velocity command using proportional control."""
        return np.clip(
            error * kp,
            -max_speed,
            max_speed
        )