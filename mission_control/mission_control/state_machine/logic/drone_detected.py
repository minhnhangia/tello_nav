#!/usr/bin/env python3
from typing import Optional

from yolo_msgs.msg import DroneDetection

from ..states import MissionState
from ..base_state import BaseState

# Lookup table: DroneDetection position constant → human-readable label.
_POSITION_LABELS = {
    DroneDetection.POSITION_LEFT: "left",
    DroneDetection.POSITION_RIGHT: "right",
}


class DroneDetectedState(BaseState):
    """Handles avoidance when another drone is detected by YOLO.

    The drone_detector_node (in yolo_ros) publishes a ``DroneDetection``
    message on the ``yolo/drone_detected`` topic.  mission_control_node
    stores the detection in the shared context and transitions here.

    Avoidance behaviour:
        - Drone on the right  → yaw **left** to avoid.
        - Drone on the left   → yaw **right** to avoid.
        - After the manoeuvre, return to SEARCHING.
    """

    def execute(self) -> Optional[MissionState]:
        """Execute DRONE_DETECTED avoidance manoeuvre.

        Sends a short yaw burst away from the detected drone, then
        returns to SEARCHING.  If the drone is still visible the
        detection callback will bring us back here for another burst.
        """
        position = self.context.drone_position

        direction = self._determine_avoidance_direction(position)

        self.node.get_logger().warning(
            f"DRONE_DETECTED: Drone at "
            f"'{_POSITION_LABELS.get(position, 'unknown')}' — "
            f"yawing {direction}"
        )

        self._execute_yaw_manoeuvre(direction)
        
        self._reset_context()

        return MissionState.SEARCHING

    def _determine_avoidance_direction(self, position: int) -> str:
        """Determine which way to yaw based on the detected drone's position."""
        # Pick direction: yaw away from where the drone is
        if position == DroneDetection.POSITION_RIGHT:
            return "left"
        else:
            # drone on left (or unknown) → yaw right
            return "right"

    def _execute_yaw_manoeuvre(self, direction: str) -> None:
        """Send the yaw command to the flight controller."""
        # --- Send yaw command for a short burst ---
        if direction == "left":
            self.drone.yaw_left_by_angle(
                10,
                MissionState.SEARCHING,
                MissionState.SEARCHING,
            )
            self.node.get_logger().info("DRONE_DETECTED: Turning Left.")
        else:
            self.drone.yaw_right_by_angle(
                10,
                MissionState.SEARCHING,
                MissionState.SEARCHING,
            )
            self.node.get_logger().info("DRONE_DETECTED: Turning Right.")

    def _reset_context(self) -> None:
        """Clear the context flags before transitioning out of the state."""
        # --- Reset context and return to SEARCHING to re-evaluate ---
        self.context.drone_position = -1
        self.context.drone_avoidance_start_time = None
        self.context.drone_avoidance_direction = ""