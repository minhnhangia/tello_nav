#!/usr/bin/env python3
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState


class SearchingState(BaseState):
    """Handles autonomous exploration with obstacle avoidance."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute SEARCHING state logic with obstacle avoidance."""
        # Wait for sensor data
        if not self.drone.has_sensor_data():
            self._wait_for_sensor_data()
            return None
        
        # 1. Exit marker avoidance
        if self.context.is_near_exit:
            self._avoid_exit()
            return None
        
        depth = self.drone.latest_depth_analysis
        is_obstacle_detected = depth.middle_center.red > depth.middle_center.blue
        is_left_clearer = depth.middle_left.blue > depth.middle_right.blue
        is_corner_detected = (depth.middle_center.blue > depth.middle_center.red and
                              self.drone.latest_tof <= self.params.corner_tof_threshold)
        is_headon_detected = self.drone.latest_tof <= self.params.headon_tof_threshold

        # 2. Obstacle Ahead (Depth Map)
        if is_obstacle_detected:
            self._avoid_obstacle(is_left_clearer)
        
        # 3. Corner Avoidance (Depth Clear + ToF Close)
        elif is_corner_detected:
            self._avoid_corner()
        
        # 4. Head-on Avoidance (ToF Very Close)
        elif is_headon_detected:
            self._avoid_headon()
        
        # 5. Path Clear - Move Forward
        else:
            self.drone.move_forward(self.params.forward_speed)
        
        return None
    
    def _wait_for_sensor_data(self):
        self.node.get_logger().debug(
            'SEARCHING: Waiting for sensor data...',
            throttle_duration_sec=5
        )
        self.drone.hover()

    def _avoid_exit(self):
        self.node.get_logger().warning(
            "SEARCHING: Near exit. Turning 90-degree clockwise."
        )
        self.context.is_near_exit = False
        self.drone.execute_action(
            'cw 90',
            MissionState.SEARCHING,
            MissionState.SEARCHING
        )

    def _avoid_obstacle(self, is_left_clearer: bool):
        if is_left_clearer:
            self.node.get_logger().warning(
                "SEARCHING: Obstacle detected. Turning Left."
            )
            self.drone.yaw_left(self.params.yaw_speed)
        else:
            self.node.get_logger().warning(
                "SEARCHING: Obstacle detected. Turning Right."
            )
            self.drone.yaw_right(self.params.yaw_speed)

    def _avoid_corner(self):
        self.node.get_logger().warning(
            "SEARCHING: Corner detected. Turning 75-degree clockwise."
        )
        self.drone.execute_action(
            'cw 75',
            MissionState.SEARCHING,
            MissionState.SEARCHING
        )

    def _avoid_headon(self):
        self.node.get_logger().warning(
            "SEARCHING: Head-on obstacle detected. Turning 180-degree clockwise."
        )
        self.drone.execute_action(
            'cw 180',
            MissionState.SEARCHING,
            MissionState.SEARCHING
        )

