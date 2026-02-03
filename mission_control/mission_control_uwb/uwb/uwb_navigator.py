"""
UWB-based navigator for waypoint navigation with position feedback.

This module provides continuous position correction using UWB localization,
integrating with the DroneInterface for ROS2-based drone control.
"""
import math
from enum import Enum, auto
from typing import Optional, TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from rclpy.node import Node
    from ..controller.drone_interface import DroneInterface


class NavPhase(Enum):
    """Internal navigation phases."""
    IDLE = auto()
    ROTATING = auto()
    MOVING = auto()
    STABILIZING = auto()


class NavStatus(Enum):
    """Status returned by update() for FSM consumption."""
    RUNNING = auto()
    SUCCESS = auto()
    BLOCKED = auto()


class UWBNavigator:
    """
    Non-blocking waypoint navigator using UWB position feedback.
    
    Designed for FSM integration - call update() each tick to progress
    toward the goal. Uses velocity commands for continuous control.
    
    Strategies:
        DRIVE: Turn to face target, then fly forward (non-holonomic)
        SLIDE: Strafe directly toward target without rotating (holonomic)
    """
    
    __slots__ = (
        'drone', 'node', 'tolerance_xy', 'tolerance_yaw', 'forward_speed',
        'strafe_speed', 'yaw_speed', 'target_pos', 'target_heading',
        'strategy', 'is_active', '_phase', '_stabilize_start_time',
        '_stabilize_duration', '_last_log_time'
    )
    
    def __init__(
        self,
        drone_interface: 'DroneInterface',
        node: 'Node',
        tolerance_xy: float = 0.20,
        tolerance_yaw: float = 5.0,
        forward_speed: float = 0.3,
        strafe_speed: float = 0.2,
        yaw_speed: float = 0.4
    ):
        """
        Initialize UWB navigator.
        
        Args:
            drone_interface: DroneInterface instance for control and telemetry
            node: ROS2 node for logging and timing
            tolerance_xy: Position tolerance in meters (default: 0.20m = 20cm)
            tolerance_yaw: Heading tolerance in degrees (default: 5.0°)
            forward_speed: Forward velocity magnitude (default: 0.3 m/s)
            strafe_speed: Strafe velocity magnitude (default: 0.2 m/s)
            yaw_speed: Yaw rate magnitude in rad/s (default: 0.4)
        """
        self.drone = drone_interface
        self.node = node
        
        # Tolerances
        self.tolerance_xy = tolerance_xy
        self.tolerance_yaw = tolerance_yaw
        
        # Speed settings
        self.forward_speed = forward_speed
        self.strafe_speed = strafe_speed
        self.yaw_speed = yaw_speed
        
        # Goal state
        self.target_pos: Optional[np.ndarray] = None  # [x, y] in meters
        self.target_heading: Optional[float] = None   # degrees
        self.strategy: str = "DRIVE"
        self.is_active: bool = False
        
        # Internal phase tracking
        self._phase: NavPhase = NavPhase.IDLE
        self._stabilize_start_time: Optional[float] = None
        self._stabilize_duration: float = 0.5  # seconds to hover for UWB settle
        self._last_log_time: float = 0.0

    def set_goal(
        self,
        x: float,
        y: float,
        heading: Optional[float] = None,
        strategy: str = "DRIVE"
    ):
        """
        Configure navigator for a new target position.
        
        Does NOT start movement immediately - call update() to execute.
        
        Args:
            x: Target X position in meters (world frame)
            y: Target Y position in meters (world frame)
            heading: Target heading in degrees (None = face target in DRIVE mode)
            strategy: "DRIVE" (turn then forward) or "SLIDE" (strafe directly)
        """
        self.target_pos = np.array([x, y], dtype=float)
        self.strategy = strategy.upper()
        self.target_heading = heading
        self.is_active = True
        self._phase = NavPhase.IDLE
        self._stabilize_start_time = None
        
        self.node.get_logger().info(
            f"[NAV] Goal set: ({x:.2f}, {y:.2f})m | Strategy: {self.strategy}"
        )

    def cancel(self):
        """Cancel current navigation and hover."""
        self.is_active = False
        self._phase = NavPhase.IDLE
        self.target_pos = None
        self.drone.hover()
        self.node.get_logger().info("[NAV] Navigation cancelled")

    def update(self) -> NavStatus:
        """
        Non-blocking tick function. Call repeatedly from FSM loop.
        
        Returns:
            NavStatus.RUNNING: Navigation in progress
            NavStatus.SUCCESS: Target reached within tolerance
            NavStatus.BLOCKED: UWB failure or navigation error
        """
        if not self.is_active:
            return NavStatus.SUCCESS
        
        # Get current pose from DroneInterface (UWB-fused)
        pose = self.drone.get_pose()
        current_pos = np.array([pose.x, pose.y])
        current_yaw = self.drone.get_heading()  # degrees
        
        # Validate UWB data (check for zero/invalid readings)
        if self._is_position_invalid(current_pos):
            self.node.get_logger().warning("[NAV] Invalid UWB position", throttle_duration_sec=2.0)
            self.drone.hover()
            return NavStatus.BLOCKED
        
        # Calculate error
        error_vector = self.target_pos - current_pos
        distance = np.linalg.norm(error_vector)
        
        # Throttled logging
        self._log_progress(distance, current_pos, current_yaw)
        
        # Check completion
        if distance < self.tolerance_xy:
            self.drone.hover()
            self.is_active = False
            self._phase = NavPhase.IDLE
            self.node.get_logger().info(
                f"[NAV] Target reached! Final error: {distance*100:.1f}cm"
            )
            return NavStatus.SUCCESS
        
        # Execute based on strategy
        if self.strategy == "SLIDE":
            self._execute_slide(error_vector, distance, current_yaw)
        else:
            self._execute_drive(error_vector, distance, current_yaw)
        
        return NavStatus.RUNNING

    # ========================================================================
    # MOVEMENT STRATEGIES
    # ========================================================================
    
    def _execute_slide(
        self,
        error_vector: np.ndarray,
        distance: float,
        current_yaw: float
    ):
        """
        Holonomic movement: strafe directly toward target.
        
        Transforms world-frame error into drone body frame and
        issues appropriate velocity commands.
        """
        # Transform error from world frame to body frame
        body_error = self._world_to_body(error_vector, current_yaw)
        
        # Normalize and scale by strafe speed
        direction = body_error / distance
        
        # Apply speed scaling based on distance (slow down when close)
        speed_scale = min(1.0, distance / 1.0)  # Ramp down within 1m
        speed = self.strafe_speed * speed_scale
        
        # Issue velocity command (x=forward, y=left in body frame)
        self.drone.move(
            x=float(direction[0] * speed),
            y=float(direction[1] * speed)
        )
        self._phase = NavPhase.MOVING

    def _execute_drive(
        self,
        error_vector: np.ndarray,
        distance: float,
        current_yaw: float
    ):
        """
        Non-holonomic movement: rotate to face target, then fly forward.
        
        This is the standard "differential drive" approach for waypoint nav.
        """
        # Calculate required heading to face target
        # Yaw convention: 0° = +Y (North), 90° = +X (East), CW-positive
        # Use atan2(x, y) to get angle from +Y axis
        target_angle_rad = math.atan2(error_vector[0], error_vector[1])
        target_angle_deg = math.degrees(target_angle_rad)
        
        # Calculate yaw error
        yaw_error = self._normalize_angle(target_angle_deg - current_yaw)
        
        # Phase 1: Rotate to face target if misaligned
        if abs(yaw_error) > self.tolerance_yaw:
            self._phase = NavPhase.ROTATING
            
            # Proportional yaw rate (faster for larger errors, min threshold)
            yaw_rate = self.yaw_speed * np.sign(yaw_error)
            if abs(yaw_error) < 20:  # Slow down for fine alignment
                yaw_rate *= abs(yaw_error) / 20.0
            
            self.drone.move(yaw=yaw_rate)
            return
        
        # Phase 2: Move forward toward target
        self._phase = NavPhase.MOVING
        
        # Speed scaling: slow down when close
        speed_scale = min(1.0, distance / 0.5)  # Ramp down within 0.5m
        speed = self.forward_speed * max(0.3, speed_scale)  # Min 30% speed
        
        self.drone.move_forward(speed)

    # ========================================================================
    # HELPERS
    # ========================================================================
    
    def _world_to_body(self, world_vec: np.ndarray, yaw_deg: float) -> np.ndarray:
        """
        Transform a 2D vector from world frame to drone body frame.
        
        Coordinate System (UPDATED):
        - World: +X is Right (East), +Y is Forward (North).
        - Yaw: 0° aligns with World +Y (North). Increases Clockwise.
        - Body: Returns [Forward, Left].
        
        Args:
            world_vec: [x, y] vector in world coordinates
            yaw_deg: Current drone heading in degrees (CW-positive)
            
        Returns:
            [forward, left] vector in body coordinates
        """
        yaw_rad = math.radians(yaw_deg)
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        
        # Derivation for World-Y=North, Yaw=0=North, CW-Positive:
        # 
        # 1. Forward Axis (Heading):
        #    - At 0° (North): Points to World +Y [0, 1]
        #    - At 90° (East): Points to World +X [1, 0]
        #    -> Axis = [sin(θ), cos(θ)]
        #
        # 2. Left Axis (90° CCW from Heading):
        #    - At 0° (North): Left is West (World -X) [-1, 0]
        #    - At 90° (East): Left is North (World +Y) [0, 1]
        #    -> Axis = [-cos(θ), sin(θ)]
        
        # Projection (Dot Product)
        body_x = world_vec[0] * sin_yaw + world_vec[1] * cos_yaw
        body_y = world_vec[0] * -cos_yaw + world_vec[1] * sin_yaw
        
        return np.array([body_x, body_y])

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Wrap angle to [-180, 180] range."""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def _is_position_invalid(self, pos: np.ndarray) -> bool:
        """
        Check if UWB position is invalid/stale.
        
        Returns True if position appears to be uninitialized or out of bounds.
        """
        # Check for uninitialized (0,0) or NaN
        if np.any(np.isnan(pos)):
            return True
        if np.allclose(pos, [0.0, 0.0], atol=1e-6):
            # (0,0) might be valid in some setups - could make configurable
            return False
        return False

    def _log_progress(self, distance: float, pos: np.ndarray, yaw: float):
        """Throttled progress logging."""
        current_time = self.node.get_clock().now().nanoseconds / 1e9
        if current_time - self._last_log_time > 1.0:  # Log every 1s
            self.node.get_logger().info(
                f"[NAV] Dist: {distance*100:.1f}cm | "
                f"Pos: ({pos[0]:.2f}, {pos[1]:.2f}) | "
                f"Yaw: {yaw}° | Phase: {self._phase.name}"
            )
            self._last_log_time = current_time

    @property
    def phase(self) -> NavPhase:
        """Current navigation phase for external monitoring."""
        return self._phase

    @property
    def distance_to_goal(self) -> Optional[float]:
        """Current distance to goal in meters, or None if no goal."""
        if self.target_pos is None:
            return None
        pose = self.drone.get_pose()
        current_pos = np.array([pose.x, pose.y])
        return float(np.linalg.norm(self.target_pos - current_pos))