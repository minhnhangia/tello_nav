#!/usr/bin/env python3
"""Mission manager containing all state execution logic."""
import math
import numpy as np
from typing import Optional
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose

from .states import MissionState
from ..controller.drone_interface import DroneInterface
from ..aruco.aruco_marker_handler import ArucoMarkerHandler
from ..utils.parameter_loader import ParameterLoader


class MissionManager:
    """
    Manages mission state logic and execution.
    
    Contains all the state-specific logic for autonomous drone operations.
    """
    
    def __init__(
        self,
        node: Node,
        drone: DroneInterface,
        marker_handler: ArucoMarkerHandler,
        params: ParameterLoader
    ):
        """
        Initialize mission manager.
        
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
        
        # Runtime state variables
        self.is_near_exit = False
        self.is_blind_landing = False
        
        # State handler dispatch table
        self.state_handlers = {
            MissionState.TAKING_OFF: self.run_taking_off_logic,
            MissionState.ASCENDING: self.run_ascending_logic,
            MissionState.SEARCHING: self.run_searching_logic,
            MissionState.CENTERING: self.run_centering_logic,
            MissionState.APPROACHING: self.run_approaching_logic,
            MissionState.CAMERA_SWITCHING: self.run_camera_switching_logic,
            MissionState.PRECISION_LANDING: self.run_precision_landing_logic,
            MissionState.LANDING: self.run_landing_logic,
            MissionState.COMPLETING_MISSION: self.run_completing_mission_logic,
            MissionState.RESETTING: self.run_resetting_logic,
        }
    
    def execute_state(self, state: MissionState) -> Optional[MissionState]:
        """
        Execute logic for the given state.
        
        Args:
            state: Current mission state to execute
            
        Returns:
            New state to transition to, or None to stay in current state
        """
        handler = self.state_handlers.get(state)
        if handler:
            return handler()
        return None
    
    def reset_runtime_state(self):
        """Reset runtime state variables."""
        self.is_near_exit = False
        self.is_blind_landing = False
    
    # ========================================================================
    # STATE EXECUTION METHODS
    # ========================================================================
    
    def run_taking_off_logic(self):
        """Execute TAKING_OFF state logic."""
        if self.drone.latest_height >= self.params.min_takeoff_height:
            self.node.get_logger().info(
                f"TAKING_OFF: Already airborne at {self.drone.latest_height:.2f}m. "
                "Transitioning to SEARCHING."
            )
            return MissionState.SEARCHING
        
        self.node.get_logger().info("TAKING_OFF: attempting takeoff.")
        self.drone.execute_action(
            'takeoff',
            MissionState.ASCENDING,
            MissionState.TAKING_OFF
        )
        return None
    
    def run_ascending_logic(self):
        """Execute ASCENDING state logic."""
        if self.drone.latest_height >= self.params.initial_search_height:
            self.node.get_logger().info(
                f"ASCENDING: Height: {self.drone.latest_height:.2f}m. "
                "Transitioning to SEARCHING."
            )
            self.drone.hover()
            return MissionState.SEARCHING
        
        self.node.get_logger().info(
            f"ASCENDING: Current height {self.drone.latest_height:.2f}m. "
            f"Ascending to {self.params.initial_search_height:.2f}m.",
            throttle_duration_sec=2.0
        )
        twist_msg = Twist()
        twist_msg.linear.z = self.params.ascending_speed
        self.drone.publish_velocity(twist_msg)
        return None
    
    def run_searching_logic(self):
        """Execute SEARCHING state logic with obstacle avoidance."""
        # Wait for sensor data
        if not self.drone.has_sensor_data():
            self.node.get_logger().info(
                'SEARCHING: Waiting for sensor data...',
                throttle_duration_sec=5
            )
            self.drone.hover()
            return None
        
        # 1. Exit marker avoidance
        if self.is_near_exit:
            self.node.get_logger().warning(
                "SEARCHING: Near exit. Turning 90-degree clockwise."
            )
            self.is_near_exit = False
            self.drone.execute_action(
                'cw 90',
                MissionState.SEARCHING,
                MissionState.SEARCHING
            )
            return None
        
        twist_msg = Twist()
        depth = self.drone.latest_depth_analysis
        
        # 2. Obstacle Ahead (Depth Map)
        if depth.middle_center.red > depth.middle_center.blue:
            if depth.middle_left.blue > depth.middle_right.blue:
                self.node.get_logger().warning(
                    "SEARCHING: Obstacle detected. Turning Left."
                )
                twist_msg.angular.z = -self.params.yaw_speed
            else:
                self.node.get_logger().warning(
                    "SEARCHING: Obstacle detected. Turning Right."
                )
                twist_msg.angular.z = self.params.yaw_speed
            self.drone.publish_velocity(twist_msg)
        
        # 3. Corner Avoidance (Depth Clear + ToF Close)
        elif (depth.middle_center.blue > depth.middle_center.red and
              self.drone.latest_tof <= self.params.corner_tof_threshold):
            self.node.get_logger().warning(
                "SEARCHING: Corner detected. Turning 150-degree clockwise."
            )
            self.drone.execute_action(
                'cw 150',
                MissionState.SEARCHING,
                MissionState.SEARCHING
            )
        
        # 4. Head-on Avoidance (ToF Very Close)
        elif self.drone.latest_tof <= self.params.headon_tof_threshold:
            self.node.get_logger().warning(
                "SEARCHING: Head-on obstacle detected. Turning 180-degree clockwise."
            )
            self.drone.execute_action(
                'cw 180',
                MissionState.SEARCHING,
                MissionState.SEARCHING
            )
        
        # 5. Path Clear - Move Forward
        else:
            twist_msg.linear.x = self.params.forward_speed
            self.drone.publish_velocity(twist_msg)
        
        return None
    
    def run_centering_logic(self):
        """Execute CENTERING state logic with obstacle avoidance and yaw alignment."""
        if self.drone.latest_depth_analysis is None:
            self.node.get_logger().warning("CENTERING: Waiting for depth analysis data...")
            return None
        
        # Check for marker timeout
        if self.marker_handler.check_marker_timeout(self.params.marker_timeout):
            self.node.get_logger().warning(
                f"CENTERING: Lost marker for >{self.params.marker_timeout}s. "
                "Returning to SEARCHING."
            )
            self.marker_handler.unreserve_current_marker()
            return MissionState.SEARCHING
        
        # Check if marker is visible
        if not self.marker_handler.is_marker_visible():
            self.node.get_logger().info(
                "CENTERING: Marker temporarily lost. Hovering...",
                throttle_duration_sec=2.0
            )
            self.drone.hover()
            return None
        
        twist_msg = Twist()
        depth_analysis = self.drone.latest_depth_analysis
        
        # --- 1. OBSTACLE AVOIDANCE (HIGHEST PRIORITY) ---
        if depth_analysis.mc_left.nonblue - 100 > depth_analysis.mc_left.blue:
            self.node.get_logger().warning(
                "CENTERING: Obstacle on the left, moving RIGHT.",
                throttle_duration_sec=1.0
            )
            twist_msg.linear.y = self.params.sideway_speed
            self.drone.publish_velocity(twist_msg)
            return None
        
        elif depth_analysis.mc_right.nonblue - 100 > depth_analysis.mc_right.blue:
            self.node.get_logger().warning(
                "CENTERING: Obstacle on the right, moving LEFT.",
                throttle_duration_sec=1.0
            )
            twist_msg.linear.y = -self.params.sideway_speed
            self.drone.publish_velocity(twist_msg)
            return None
        
        # --- 2. YAW CENTERING (RUNS IF PATH IS CLEAR) ---
        marker_pose: Pose = self.marker_handler.get_locked_marker_pose()
        x_error = marker_pose.position.x
        forward_dist = marker_pose.position.z
        
        # Check if centered and close enough to approach
        if (abs(x_error) < self.params.centering_threshold_x and
            forward_dist < self.params.max_approach_dist):
            self.node.get_logger().info(
                "CENTERING: Centered on marker. Transitioning to APPROACHING."
            )
            self.drone.hover()
            return MissionState.APPROACHING
        
        # Centered but far - move forward
        if (abs(x_error) < self.params.centering_threshold_x and
            forward_dist >= self.params.max_approach_dist):
            self.node.get_logger().info(
                f"CENTERING: Centered but far from marker ({forward_dist:.2f}m). "
                "Moving forward!"
            )
            self.drone.execute_action(
                f'forward {self.params.step_approach_dist * 100}',
                MissionState.CENTERING,
                MissionState.CENTERING
            )
            return None
        
        # Not centered - apply yaw correction
        if abs(x_error) >= self.params.centering_threshold_x:
            yaw_speed = np.clip(
                x_error * self.params.centering_yaw_kp,
                -self.params.yaw_speed,
                self.params.yaw_speed
            )
            twist_msg.angular.z = yaw_speed
        
        # Also move forward if far
        if forward_dist > self.params.max_approach_dist:
            twist_msg.linear.x = self.params.forward_speed
        
        self.node.get_logger().info(
            f"CENTERING: x_error: {x_error:.2f}m, forward_dist: {forward_dist:.2f}m, "
            f"yaw_speed: {twist_msg.angular.z:.2f} forward_speed: {twist_msg.linear.x:.2f}",
            throttle_duration_sec=1.0
        )
        self.drone.publish_velocity(twist_msg)
        return None
    
    def run_approaching_logic(self):
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
            self.node.get_logger().info(
                "APPROACHING: Marker temporarily lost. Hovering...",
                throttle_duration_sec=2.0
            )
            self.drone.hover()
            return None
        
        marker_pose: Pose = self.marker_handler.get_locked_marker_pose()
        x = marker_pose.position.x
        y = marker_pose.position.y
        z = marker_pose.position.z
        
        forward_dist = z
        
        self.node.get_logger().info(
            f"APPROACHING: moving forward {forward_dist:.2f} m, "
            f"x = {x:.2f} m, y = {y:.2f} m"
        )
        forward_dist_cmd = max(
            0,
            int(forward_dist * 100 - self.params.final_approach_offset * 100)
        )
        self.drone.execute_action(
            f'forward {forward_dist_cmd}',
            MissionState.CAMERA_SWITCHING,
            MissionState.CENTERING
        )
        return None
    
    def run_camera_switching_logic(self):
        """Execute CAMERA_SWITCHING state logic."""
        self.node.get_logger().info(
            "Switching to downward-facing camera for precision landing."
        )
        self.drone.execute_action(
            'downvision 1',
            MissionState.PRECISION_LANDING,
            MissionState.CAMERA_SWITCHING
        )
        return None
    
    def run_precision_landing_logic(self):
        """Execute PRECISION_LANDING state logic with recovery maneuvers."""
        # Check for full timeout - land blind
        if self.marker_handler.check_marker_timeout(self.params.precision_landing_timeout):
            self.node.get_logger().warning(
                f"PRECISION_LANDING: Lost marker for >{self.params.precision_landing_timeout}s. "
                "Landing blind!"
            )
            self.drone.hover()
            self.is_blind_landing = True
            return MissionState.LANDING
        
        # Check for partial timeout - recover altitude if low
        if self.marker_handler.check_marker_timeout(self.params.precision_landing_timeout / 2):
            if self.drone.latest_height < self.params.recovery_height:
                self.node.get_logger().warning(
                    f"PRECISION_LANDING: Lost marker for "
                    f">{self.params.precision_landing_timeout/2}s. Moving up!",
                    throttle_duration_sec=2.0
                )
                twist_msg = Twist()
                twist_msg.linear.z = self.params.ascending_speed
                self.drone.publish_velocity(twist_msg)
                return None
        
        # Check if marker is visible
        if not self.marker_handler.is_marker_visible():
            self.node.get_logger().info(
                "PRECISION_LANDING: Marker temporarily lost. Hovering...",
                throttle_duration_sec=2.0
            )
            self.drone.hover()
            return None
        
        twist_msg = Twist()
        
        # Extract positional errors
        marker_pose: Pose = self.marker_handler.get_locked_marker_pose()
        y_error = marker_pose.position.y
        x_error = marker_pose.position.x
        
        # Check if centered on both axes
        x_centered = abs(x_error) <= self.params.precision_landing_threshold_x
        y_centered = abs(y_error) <= self.params.precision_landing_threshold_y
        
        if x_centered and y_centered:
            self.node.get_logger().info(
                "PRECISION_LANDING: Centered over marker. Transitioning to LANDING."
            )
            self.drone.hover()
            self.is_blind_landing = False
            return MissionState.LANDING
        
        # Proportional control for final alignment
        if not x_centered:
            forward_speed = np.clip(
                -x_error * self.params.precision_forward_kp,
                -self.params.precision_landing_max_speed,
                self.params.precision_landing_max_speed
            )
            twist_msg.linear.x = forward_speed
        
        if not y_centered:
            sideway_speed = np.clip(
                -y_error * self.params.precision_sideway_kp,
                -self.params.precision_landing_max_speed,
                self.params.precision_landing_max_speed
            )
            twist_msg.linear.y = sideway_speed
        
        self.node.get_logger().info(
            f"PRECISION_LANDING: x_err: {x_error:.2f}, y_err: {y_error:.2f}, "
            f"speed: {twist_msg.linear}",
            throttle_duration_sec=1.0
        )
        self.drone.publish_velocity(twist_msg)
        return None
    
    def run_landing_logic(self):
        """Execute LANDING state logic."""
        if self.drone.latest_height < self.params.min_takeoff_height:
            self.node.get_logger().info(
                f"LANDING: Already landed at {self.drone.latest_height:.2f}m. "
                "Transitioning to COMPLETING_MISSION."
            )
            return MissionState.COMPLETING_MISSION
        
        self.drone.execute_action(
            "land",
            MissionState.COMPLETING_MISSION,
            MissionState.LANDING
        )
        return None
    
    def run_completing_mission_logic(self):
        """Execute COMPLETING_MISSION state logic."""
        if not self.is_blind_landing:
            self.node.get_logger().info(
                "COMPLETING_MISSION: Marking marker as landed and completing mission."
            )
            self.marker_handler.mark_current_marker_landed()
        else:
            self.node.get_logger().warning(
                "COMPLETING_MISSION: Blind landing. NOT marking marker as landed."
            )
        return MissionState.RESETTING
    
    def run_resetting_logic(self):
        """Execute RESETTING state logic."""
        self.node.get_logger().info("Resetting Mission Control to initial state.")
        self.reset_runtime_state()
        self.marker_handler.reset_marker_state()
        self.drone.execute_action(
            'downvision 0',
            MissionState.IDLE,
            MissionState.IDLE
        )
        return None
