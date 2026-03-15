#!/usr/bin/env python3
"""
Exit Marker Handler Module
"""

# import rclpy
# from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection

from typing import Set
import numpy as np


class ExitMarkerHandler:
    
    def __init__(
        self,
        exit_markers: Set[int],
    ):
        """
        Initialize the Exit Marker Handler.
        
        Args:
            exit_markers: Set of exit marker IDs
        """
        self.exit_markers = exit_markers

    def is_near_exit_marker(self, msg: ArucoDetection) -> bool:
        """
        Check if near any exit marker.
        
        Args:
            msg: ArucoDetection message containing detected markers

        Returns:
            True if near any exit marker, False otherwise.
        """
        for marker in msg.markers:
            if marker.marker_id in self.exit_markers:
                x = marker.pose.position.x
                y = marker.pose.position.y
                z = marker.pose.position.z
                distance = np.sqrt(x**2 + y**2 + z**2)
                if distance < 5.0:  # 5 meters threshold
                    return True
        return False