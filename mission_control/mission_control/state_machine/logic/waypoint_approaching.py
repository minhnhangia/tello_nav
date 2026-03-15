#!/usr/bin/env python3
from typing import Optional

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
        state = self._validate_waypoint_state()
        if state is not None:
            return state

        # ====================================================================
        # PHASE 1: SWITCH TO DOWNWARD CAMERA
        # ====================================================================
        if not self.context.camera_switched:
            return self._handle_camera_switch()

        # ====================================================================
        # PHASE 2: CHECK IF DOWNWARD CAMERA DETECTED MARKER
        # ====================================================================
        if self.context.downward_camera_active:
            return self._handle_downward_detection()

        # ====================================================================
        # PHASE 3: IF FORWARD CAMERA IS ACTIVE, TRANSITION TO NEXT STATE
        # ====================================================================
        return self._handle_forward_camera_transition()

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    def _validate_waypoint_state(self) -> Optional[MissionState]:
        """Validate waypoint manager and sequence state."""

        if self.waypoint_manager is None:
            self.node.get_logger().error(
                "WAYPOINT_APPROACHING: Waypoint manager unavailable. Transitioning to SEARCHING."
            )
            return MissionState.SEARCHING

        # Check if waypoint sequence is complete
        if self.waypoint_manager.is_sequence_complete():
            self.node.get_logger().error(
                "WAYPOINT_APPROACHING: Waypoint sequence complete. "
                "Transitioning to SEARCHING."
            )
            return MissionState.SEARCHING
        
        # Defensive check: action-only waypoints should not reach this state
        if self.waypoint_manager.is_current_waypoint_action_only():
            self.node.get_logger().warning(
                "WAYPOINT_APPROACHING: Action-only waypoint detected. "
                "Transitioning to WAYPOINT_ACTION."
            )
            return MissionState.WAYPOINT_ACTION

        return None

    # ------------------------------------------------------------------
    # PHASE 1
    # ------------------------------------------------------------------

    def _handle_camera_switch(self) -> Optional[MissionState]:
        """Handle switching to downward camera."""

        self.node.get_logger().info(
            "WAYPOINT_APPROACHING: Switching to downward camera and starting forward movement."
        )

        self.drone.execute_action(
            'downvision 1',
            MissionState.WAYPOINT_APPROACHING,
            MissionState.SEARCHING,
            max_retries=5
        )

        self.context.camera_switched = True
        self.context.downward_camera_active = True
        self.context.marker_detected_with_down_camera = False

        self.waypoint_manager.clear_marker_pose()

        # START MOVING FORWARD IMMEDIATELY
        self._move_forward_with_downward_camera()

        return None

    # ------------------------------------------------------------------
    # PHASE 2
    # ------------------------------------------------------------------

    def _handle_downward_detection(self) -> Optional[MissionState]:
        """Handle marker detection using downward camera."""

        self._move_forward_with_downward_camera()

        # Check if marker was detected with downward camera
        if not self.waypoint_manager.is_marker_visible_on_downward_camera():
            return None

        detection_streak = self.waypoint_manager.get_downward_detection_streak()
        required_streak = self.params.waypoint_downward_detection_frames

        if detection_streak < required_streak:
            self.node.get_logger().debug(
                "WAYPOINT_APPROACHING: Downward marker streak "
                f"{detection_streak}/{required_streak}; waiting for confirmation.",
                throttle_duration_sec=1.0
            )
            return None

        self.node.get_logger().info(
            "WAYPOINT_APPROACHING: Downward camera confirmed marker "
            f"for {detection_streak} consecutive frames. Switching back to forward camera."
        )

        self.context.marker_detected_with_down_camera = True

        self.drone.hover()  # Stop moving while switching cameras

        # Switch back to forward camera
        self.drone.execute_action(
            'downvision 0',
            MissionState.WAYPOINT_APPROACHING,
            MissionState.SEARCHING,
            max_retries=5
        )

        self.context.downward_camera_active = False

        return None

    # ------------------------------------------------------------------
    # PHASE 3
    # ------------------------------------------------------------------

    def _handle_forward_camera_transition(self) -> Optional[MissionState]:
        """Handle transition once forward camera becomes active."""

        if self.context.marker_detected_with_down_camera and not self.context.downward_camera_active:
            self.node.get_logger().info(
                "WAYPOINT_APPROACHING: Forward camera active. Transitioning to WAYPOINT_ACTION."
            )
            return MissionState.WAYPOINT_ACTION

        return None

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def _move_forward_with_downward_camera(self):
        """Publish continuous forward velocity while downward camera is active."""
        forward_speed = self.params.centering_forward_speed
        self.drone.move_forward(forward_speed)
        
        self.node.get_logger().debug(
            "WAYPOINT_APPROACHING: Moving forward with downward camera active (waiting for marker detection)...",
            throttle_duration_sec=2.0
        )