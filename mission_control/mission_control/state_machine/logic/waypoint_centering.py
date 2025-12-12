#!/usr/bin/env python3
import numpy as np
from typing import Optional
from geometry_msgs.msg import Twist, Pose
from midas_msgs.msg import DepthMapAnalysis

from ..states import MissionState
from ..base_state import BaseState


class WaypointCenteringState(BaseState):
    """Handles yaw alignment on waypoint marker with obstacle avoidance."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute WAYPOINT_CENTERING state logic with obstacle avoidance and yaw alignment."""
        # Check waypoint manager availability
        if self.waypoint_manager is None or self.waypoint_manager.is_sequence_complete():
            self.node.get_logger().error(
                "WAYPOINT_CENTERING: No waypoint manager or sequence complete. "
                "Transitioning to SEARCHING."
            )
            return MissionState.SEARCHING
        
        # Check for marker timeout
        if self.waypoint_manager.check_marker_timeout(self.params.waypoint_timeout):
            self._handle_permanent_marker_lost()
            return MissionState.SEARCHING
        
        if not self.drone.has_sensor_data():
            self._wait_for_sensor_data()
            return None
        
        # Get sensor data
        depth: DepthMapAnalysis = self.drone.latest_depth_analysis # type: ignore
        tof: float = self.drone.latest_tof # type: ignore
        
        # Check if marker is visible
        if not self.waypoint_manager.is_marker_visible():
            self._handle_temporary_marker_lost(depth, tof)
            return None
        
        marker_pose: Pose = self.waypoint_manager.get_marker_pose() # type: ignore
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
        if is_centered and is_close_enough:
            self._handle_successful_centering()
            return MissionState.WAYPOINT_APPROACHING
        
        # Centered but far - move closer
        if is_centered and not is_close_enough:
            self._move_closer_to_marker(forward_dist)
            return None
    
    def _wait_for_sensor_data(self):
        self.node.get_logger().info(
            'WAYPOINT_CENTERING: Waiting for sensor data...',
            throttle_duration_sec=5
        )
        self.drone.hover()
    
    def _handle_permanent_marker_lost(self):
        current_wp = self.waypoint_manager.get_current_waypoint() if self.waypoint_manager else None
        marker_id = current_wp.marker_id if current_wp else "unknown"
        self.node.get_logger().warning(
            f"WAYPOINT_CENTERING: Lost waypoint marker {marker_id} for "
            f">{self.params.waypoint_timeout}s. Aborting to SEARCHING."
        )
    
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
        
        if is_safe_to_move_forward:
            self.node.get_logger().info(
                "WAYPOINT_CENTERING: Marker temporarily lost. Moving forward to find...",
                throttle_duration_sec=2.0
            )
            self.drone.move_forward(self.params.forward_speed)
        else:
            self._hover_on_temporary_marker_lost()
    
    def _hover_on_temporary_marker_lost(self):
        self.node.get_logger().info(
            "WAYPOINT_CENTERING: Marker temporarily lost. Hovering...",
            throttle_duration_sec=2.0
        )
        self.drone.hover()
    
    def _perform_yaw_centering(self, x_error: float):
        current_wp = self.waypoint_manager.get_current_waypoint() if self.waypoint_manager else None
        marker_id = current_wp.marker_id if current_wp else "unknown"
        
        twist_msg = Twist()
        twist_msg.angular.z = self._compute_speed(
            error=x_error,
            kp=self.params.centering_yaw_kp,
            max_speed=self.params.centering_yaw_speed
        )
        self.node.get_logger().info(
            f"WAYPOINT_CENTERING: Marker {marker_id}, x_error: {x_error:.2f}m, "
            f"yaw_speed: {twist_msg.angular.z:.2f}",
            throttle_duration_sec=1.0
        )
        self.drone.publish_velocity(twist_msg)
    
    def _avoid_obstacle_if_needed(self, depth: DepthMapAnalysis, 
                                  x_error: float) -> bool:
        is_obstacle_left = depth.mc_left.nonblue - 100 > depth.mc_left.blue
        is_obstacle_right = depth.mc_right.nonblue - 100 > depth.mc_right.blue
        is_marker_left = x_error <= -self.params.centering_threshold_x
        is_marker_right = x_error >= self.params.centering_threshold_x
        
        if is_obstacle_left and not is_marker_left:
            self.node.get_logger().warning(
                "WAYPOINT_CENTERING: Obstacle on the left, marker on the right, moving RIGHT.",
                throttle_duration_sec=1.0
            )
            self.drone.move_right(self.params.sideway_speed)
            return True
        elif is_obstacle_right and not is_marker_right:
            self.node.get_logger().warning(
                "WAYPOINT_CENTERING: Obstacle on the right, marker on the left, moving LEFT.",
                throttle_duration_sec=1.0
            )
            self.drone.move_left(self.params.sideway_speed)
            return True
        
        return False
    
    def _handle_successful_centering(self):
        current_wp = self.waypoint_manager.get_current_waypoint() if self.waypoint_manager else None
        marker_id = current_wp.marker_id if current_wp else "unknown"
        self.node.get_logger().info(
            f"WAYPOINT_CENTERING: Centered on waypoint marker {marker_id}. "
            "Transitioning to WAYPOINT_APPROACHING."
        )
        self.drone.hover()
    
    def _move_closer_to_marker(self, forward_dist: float):
        """Move closer to the marker by a fixed step distance."""
        step_distance = self.params.step_approach_dist
        current_wp = self.waypoint_manager.get_current_waypoint() if self.waypoint_manager else None
        marker_id = current_wp.marker_id if current_wp else "unknown"
        
        self.node.get_logger().info(
            f"WAYPOINT_CENTERING: Centered but far from waypoint {marker_id} "
            f"({forward_dist:.2f}m). Moving closer by {step_distance:.2f}m."
        )
        self.drone.execute_action(
            f'forward {int(step_distance * 100)}',
            MissionState.WAYPOINT_CENTERING,
            MissionState.WAYPOINT_CENTERING
        )
    
    def _compute_speed(self, error: float, kp: float, max_speed: float) -> float:
        """Compute velocity command using proportional control."""
        return np.clip(
            error * kp,
            -max_speed,
            max_speed
        )