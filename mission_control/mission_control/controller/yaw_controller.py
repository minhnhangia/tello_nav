#!/usr/bin/env python3
"""Yaw controller for precise angle-based rotation using velocity commands."""

from enum import Enum
from typing import TYPE_CHECKING, Callable, Optional

from rclpy.node import Node
from geometry_msgs.msg import Twist

if TYPE_CHECKING:
    from ..state_machine.states import MissionState


class YawState(Enum):
    """Defines the internal states of the YawController."""
    IDLE = 0
    ROTATING = 1


class YawController:
    """
    A dedicated controller for precise angle-based yaw rotation.
    
    Uses continuous velocity commands with heading feedback to rotate
    the drone by a specified angle, avoiding the unreliable Tello SDK
    'cw'/'ccw' commands.
    
    Integrates with the main control loop's polling pattern - call
    check_progress() at regular intervals (e.g., 4 Hz) to monitor
    rotation completion.
    """
    
    # Default parameters
    DEFAULT_TIMEOUT_SEC = 10.0
    DEFAULT_TOLERANCE_DEG = 5.0
    DEFAULT_YAW_SPEED = 0.6  # rad/s
    SLOWDOWN_THRESHOLD_DEG = 30.0  # Start slowing down within this range
    MIN_YAW_SPEED = 0.2  # Minimum yaw speed during slowdown
    
    def __init__(
        self,
        node: Node,
        cmd_vel_publisher,
        get_heading_callback: Callable[[], int],
        on_success_callback: Callable,
        on_fail_callback: Callable,
        on_execute_callback: Callable
    ):
        """
        Initialize the YawController.
        
        Args:
            node: ROS2 node for logging and timing
            cmd_vel_publisher: Publisher for Twist messages (cmd_vel)
            get_heading_callback: Callable returning current heading in degrees
            on_success_callback: Called with next_state on successful rotation
            on_fail_callback: Called with fallback_state on failure/timeout
        """
        self.node = node
        self.logger = self.node.get_logger()
        self.cmd_vel_pub = cmd_vel_publisher
        self.get_heading = get_heading_callback
        
        # State transition callbacks
        self.on_success_callback = on_success_callback
        self.on_fail_callback = on_fail_callback
        self.on_execute_callback = on_execute_callback
        
        # Controller state
        self.state = YawState.IDLE
        
        # Rotation parameters (set per rotation)
        self.target_heading: Optional[float] = None
        self.rotation_direction: int = 0  # +1 for CW (right), -1 for CCW (left)
        self.pending_next_state: Optional['MissionState'] = None
        self.pending_fallback_state: Optional['MissionState'] = None
        self.current_timeout: float = self.DEFAULT_TIMEOUT_SEC
        self.current_tolerance: float = self.DEFAULT_TOLERANCE_DEG
        self.current_speed: float = self.DEFAULT_YAW_SPEED
        
        # Timing
        self.rotation_start_time = None
    
    def is_busy(self) -> bool:
        """Check if a rotation is currently in progress."""
        return self.state != YawState.IDLE
    
    def yaw_right_by_angle(
        self,
        angle: float,
        next_state: 'MissionState',
        fallback_state: 'MissionState',
        speed: Optional[float] = None,
        timeout: Optional[float] = None,
        tolerance: Optional[float] = None
    ):
        """
        Rotate clockwise (right) by specified angle.
        
        Args:
            angle: Degrees to rotate (positive value)
            next_state: State to transition to on success
            fallback_state: State to transition to on failure
            speed: Yaw rate in rad/s (default: 0.5)
            timeout: Timeout in seconds (default: 15.0)
            tolerance: Completion tolerance in degrees (default: 5.0)
        """
        self._start_rotation(
            angle=abs(angle),
            direction=1,  # CW = positive yaw rate
            next_state=next_state,
            fallback_state=fallback_state,
            speed=speed,
            timeout=timeout,
            tolerance=tolerance
        )
    
    def yaw_left_by_angle(
        self,
        angle: float,
        next_state: 'MissionState',
        fallback_state: 'MissionState',
        speed: Optional[float] = None,
        timeout: Optional[float] = None,
        tolerance: Optional[float] = None
    ):
        """
        Rotate counter-clockwise (left) by specified angle.
        
        Args:
            angle: Degrees to rotate (positive value)
            next_state: State to transition to on success
            fallback_state: State to transition to on failure
            speed: Yaw rate in rad/s (default: 0.5)
            timeout: Timeout in seconds (default: 15.0)
            tolerance: Completion tolerance in degrees (default: 5.0)
        """
        self._start_rotation(
            angle=abs(angle),
            direction=-1,  # CCW = negative yaw rate
            next_state=next_state,
            fallback_state=fallback_state,
            speed=speed,
            timeout=timeout,
            tolerance=tolerance
        )
    
    def _start_rotation(
        self,
        angle: float,
        direction: int,
        next_state: 'MissionState',
        fallback_state: 'MissionState',
        speed: Optional[float] = None,
        timeout: Optional[float] = None,
        tolerance: Optional[float] = None
    ):
        """Internal method to initiate a rotation."""
        if self.is_busy():
            self.logger.warning(
                f"YawController is busy, cannot start rotation"
            )
            return
        
        # Store parameters
        self.rotation_direction = direction
        self.pending_next_state = next_state
        self.pending_fallback_state = fallback_state
        self.current_speed = speed if speed is not None else self.DEFAULT_YAW_SPEED
        self.current_timeout = timeout if timeout is not None else self.DEFAULT_TIMEOUT_SEC
        self.current_tolerance = tolerance if tolerance is not None else self.DEFAULT_TOLERANCE_DEG
        
        # Compute target heading
        current_heading = float(self.get_heading())
        # CW (right) increases heading, CCW (left) decreases heading
        delta = angle * direction
        self.target_heading = self._normalize_angle(current_heading + delta)
        
        # Start rotation
        self.state = YawState.ROTATING
        self.rotation_start_time = self.node.get_clock().now()
        
        direction_str = "CW (right)" if direction > 0 else "CCW (left)"
        self.logger.info(
            f"Starting {direction_str} rotation: {angle:.1f}° "
            f"(current: {current_heading:.1f}°, target: {self.target_heading:.1f}°)"
        )
        
        # Send initial yaw command
        self._send_yaw_command()
    
    def check_progress(self):
        """
        Check rotation progress. Call this from the main control loop.
        
        This method:
        1. Checks if rotation is complete (within tolerance)
        2. Checks for timeout
        3. Adjusts yaw speed based on remaining angle (slowdown near target)
        4. Continues sending yaw commands if still rotating
        """
        if self.state != YawState.ROTATING:
            return
        
        if self.target_heading is None:
            self.logger.error("check_progress called but target_heading is None")
            self._complete_rotation(success=False)
            return
        
        current_heading = float(self.get_heading())
        remaining = self._angular_distance(current_heading, self.target_heading)
        remaining_abs = abs(remaining)
        
        # Check completion
        if remaining_abs <= self.current_tolerance:
            self._complete_rotation(success=True)
            return
        
        # Check timeout
        if self.rotation_start_time is not None:
            elapsed = (self.node.get_clock().now() - self.rotation_start_time).nanoseconds / 1e9
            if elapsed > self.current_timeout:
                self.logger.error(
                    f"Yaw rotation timed out after {elapsed:.1f}s "
                    f"(remaining: {remaining_abs:.1f}°)"
                )
                self._complete_rotation(success=False)
                return
        
        # Continue rotation with adaptive speed
        self._send_yaw_command(remaining_abs)
    
    def _send_yaw_command(self, remaining_angle: Optional[float] = None):
        """Send yaw velocity command with optional adaptive slowdown."""
        # Calculate speed with slowdown near target
        speed = self.current_speed
        if remaining_angle is not None and remaining_angle < self.SLOWDOWN_THRESHOLD_DEG:
            # Linear interpolation for smooth slowdown
            ratio = remaining_angle / self.SLOWDOWN_THRESHOLD_DEG
            speed = self.MIN_YAW_SPEED + (self.current_speed - self.MIN_YAW_SPEED) * ratio
        
        twist = Twist()
        twist.angular.z = speed * self.rotation_direction
        self.cmd_vel_pub.publish(twist)
    
    def _complete_rotation(self, success: bool):
        """Complete the rotation and trigger appropriate callback."""
        # Stop rotation
        self.cmd_vel_pub.publish(Twist())

        self.on_execute_callback()
        
        if success:
            final_heading = float(self.get_heading())
            self.logger.info(
                f"Yaw rotation complete (final heading: {final_heading:.1f}°)"
            )
            next_state = self.pending_next_state
            self.reset()
            self.on_success_callback(next_state)
        else:
            fallback_state = self.pending_fallback_state
            self.reset()
            self.on_fail_callback(fallback_state)
    
    def cancel(self):
        """Cancel any ongoing rotation without triggering callbacks."""
        if self.is_busy():
            self.logger.info("Yaw rotation cancelled")
            self.cmd_vel_pub.publish(Twist())  # Stop rotation
            self.on_execute_callback()
            self.reset()
    
    def reset(self):
        """Reset controller to idle state."""
        self.state = YawState.IDLE
        self.target_heading = None
        self.rotation_direction = 0
        self.pending_next_state = None
        self.pending_fallback_state = None
        self.rotation_start_time = None
        self.current_timeout = self.DEFAULT_TIMEOUT_SEC
        self.current_tolerance = self.DEFAULT_TOLERANCE_DEG
        self.current_speed = self.DEFAULT_YAW_SPEED
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """
        Normalize angle to [-180, 180] range.
        
        Args:
            angle: Angle in degrees
            
        Returns:
            Normalized angle in degrees
        """
        while angle > 180:
            angle -= 360
        while angle <= -180:
            angle += 360
        return angle
    
    def _angular_distance(self, current: float, target: float) -> float:
        """
        Compute shortest angular distance from current to target.
        
        Args:
            current: Current heading in degrees
            target: Target heading in degrees
            
        Returns:
            Signed angular distance (positive = CW, negative = CCW)
        """
        diff = target - current
        return self._normalize_angle(diff)
