#!/usr/bin/env python3
"""
ArUco Marker Handler Module

Manages all ArUco marker detection, selection, and coordination logic
for the Mission Control system.
"""

import rclpy
from rclpy.node import Node
from typing import Optional, Set, Callable
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32MultiArray
from swarm_interfaces.srv import ReserveMarker, MarkLanded


class ArucoMarkerHandler:
    """
    Handles ArUco marker detection, selection, and swarm coordination.
    
    Responsibilities:
    - Process incoming ArUco detections
    - Select best available marker based on priority and availability
    - Manage marker reservations with the swarm coordinator
    - Track locked marker state and visibility
    """
    
    def __init__(
        self,
        node: Node,
        drone_id: str,
        priority_markers: Set[int],
        on_marker_locked_callback: Callable,
        on_marker_lost_callback: Callable
    ):
        """
        Initialize the ArUco Marker Handler.
        
        Args:
            node: Parent ROS2 node for logging and service clients
            drone_id: Unique identifier for this drone
            priority_markers: Set of high-priority marker IDs
            on_marker_locked_callback: Called when marker successfully locked (marker_id)
            on_marker_lost_callback: Called when marker timeout expires
        """
        self.node = node
        self.logger = node.get_logger()
        self.drone_id = drone_id
        self.priority_markers = priority_markers
        
        # Callbacks to notify mission control
        self.on_marker_locked = on_marker_locked_callback
        self.on_marker_lost = on_marker_lost_callback
        
        # Marker state tracking
        self.locked_on_marker_id = -1
        self.locked_on_marker_pose: Optional[Pose] = None
        self.marker_last_seen_time = node.get_clock().now()
        self.unavailable_markers: Set[int] = set()
        
        # Setup subscriptions and service clients
        self._setup_communication()
        
        self.logger.info("ArUco Marker Handler initialized.")
    
    def _setup_communication(self):
        """Setup ROS2 subscriptions and service clients."""
        # Subscribe to unavailable markers from the swarm coordinator
        self.unavailable_markers_sub = self.node.create_subscription(
            Int32MultiArray,
            '/unavailable_markers',
            self._unavailable_markers_callback,
            10
        )
        
        # Service clients for marker coordination
        self.request_marker_client = self.node.create_client(
            ReserveMarker, '/reserve_marker')
        while not self.request_marker_client.wait_for_service(timeout_sec=1.0):
            self.logger.info('Reserve Marker service not available, waiting...')
        
        self.unreserve_marker_client = self.node.create_client(
            ReserveMarker, '/unreserve_marker')
        while not self.unreserve_marker_client.wait_for_service(timeout_sec=1.0):
            self.logger.info('Unreserve Marker service not available, waiting...')
        
        self.mark_landed_client = self.node.create_client(
            MarkLanded, '/mark_landed')
        while not self.mark_landed_client.wait_for_service(timeout_sec=1.0):
            self.logger.info('Mark Landed service not available, waiting...')
    
    def _unavailable_markers_callback(self, msg: Int32MultiArray):
        """Update the list of unavailable markers from the swarm coordinator."""
        self.unavailable_markers = set(msg.data)

    def _find_priority_marker(self, avail_markers) -> Optional[object]:
        priority_markers_avail = [
            m for m in avail_markers
            if m.marker_id in self.priority_markers
        ]

        if priority_markers_avail:
            # Found priority marker(s) - select closest one
            priority_marker = min(priority_markers_avail, key=lambda m: m.pose.position.z)
            return priority_marker

        return None
    
    def _filter_unavailable_markers(self, markers) -> list:
        return [
            m for m in markers 
            if m.marker_id not in self.unavailable_markers
        ]
    
    def select_best_marker(self, markers) -> Optional[object]:
        """
        Selects the most suitable marker from detected markers.
        
        Selection Strategy (in priority order):
        1. Filter out unavailable markers (already reserved/landed by other drones)
        2. If priority markers are detected and available, return the closest priority marker
        3. Otherwise, return the closest available marker
        
        Args:
            markers: List of detected ArUco markers from ArucoDetection message
            
        Returns:
            The best marker object, or None if no suitable markers found
        """
        if not markers:
            return None
        
        # Step 1: Filter out unavailable markers
        available_markers = self._filter_unavailable_markers(markers)
        
        if not available_markers:
            self.logger.info(
                f"All {len(markers)} detected marker(s) are unavailable: "
                f"{[m.marker_id for m in markers]}", 
                throttle_duration_sec=2.0
            )
            return None
        
        # If only one available marker, return it immediately
        if len(available_markers) == 1:
            return available_markers[0]
        
        # Step 2: Check for priority markers
        priority_marker = self._find_priority_marker(available_markers)
        if priority_marker:
            self.logger.warning(
                f"Selected PRIORITY marker {priority_marker.marker_id}"
            )
            return priority_marker
        
        # Step 3: No priority markers - select closest available marker
        best_marker = min(available_markers, key=lambda m: m.pose.position.z)
        distance = best_marker.pose.position.z
        self.logger.info(
            f"Selected closest marker {best_marker.marker_id} at {distance:.2f}m "
            f"from {len(available_markers)} available marker(s)"
        )
        
        return best_marker

    def should_switch_to_priority_marker(self, msg: ArucoDetection) -> bool:
        """
        Decide whether to switch to a priority marker while already tracking another marker.

        Args:
            msg: ArucoDetection message containing detected markers

        Returns:
            True if a priority marker is available, False otherwise
        """
        if self.locked_on_marker_id in self.priority_markers:
            # Already locked on a priority marker
            return False
        
        markers = msg.markers
        if len(markers) == 0:
            return False
        
        available_markers = self._filter_unavailable_markers(markers)
        if not available_markers:
            return False

        for marker in available_markers:
            if marker.marker_id in self.priority_markers:
                self.logger.warning(
                    f"Priority marker {marker.marker_id} detected while tracking "
                    f"marker {self.locked_on_marker_id}."
                )
                return True
            
        return False

    def process_aruco_detection_for_search(self, msg: ArucoDetection) -> bool:
        """
        Process ArUco detection during SEARCHING state.
        
        Attempts to select and lock onto a suitable marker.
        
        Args:
            msg: ArucoDetection message containing detected markers
            
        Returns:
            True if marker lock initiated, False otherwise
        """
        if len(msg.markers) == 0:
            return False
        
        try:
            best_marker = self.select_best_marker(msg.markers)
            if best_marker is None:
                # All detected markers are unavailable
                return False
            
            # Found a suitable marker - attempt to lock on
            detected_id = best_marker.marker_id
            self.locked_on_marker_pose = best_marker.pose
            self.marker_last_seen_time = self.node.get_clock().now()
            
            self.logger.info(f"Marker {detected_id} selected. Attempting to lock on...")
            self.request_marker_lock(detected_id, handle_response=True)
            return True
            
        except Exception as e:
            self.logger.error(f"Error processing ArUco detection: {e}")
            return False
    
    def update_locked_marker_pose(self, msg: ArucoDetection):
        """
        Update the pose of the currently locked marker.
        
        Call this during CENTERING, APPROACHING, or PRECISION_LANDING states.
        
        Args:
            msg: ArucoDetection message containing detected markers
        """
        marker_found = False
        for marker in msg.markers:
            if marker.marker_id == self.locked_on_marker_id:
                self.locked_on_marker_pose = marker.pose
                marker_found = True
                self.marker_last_seen_time = self.node.get_clock().now()
                break
        
        # If the locked-on marker is no longer visible, set pose to None
        if not marker_found:
            self.locked_on_marker_pose = None

    def check_marker_timeout(self, duration: float = 2.0) -> bool:
        """
        Check if the locked marker has timed out.
        
        Returns:
            True if marker has timed out, False otherwise
        """
        if self.locked_on_marker_id < 0:
            return False
        
        time_since_last_marker = (
            self.node.get_clock().now() - self.marker_last_seen_time
        ).nanoseconds / 1e9

        return time_since_last_marker > duration

    def request_marker_lock(self, marker_id: int, handle_response: bool = True):
        """
        Request to reserve a marker from the swarm coordinator.
        
        Args:
            marker_id: ID of the marker to reserve
            handle_response: Whether to handle the async response with callback
        """
        req = ReserveMarker.Request()
        req.drone_id = self.drone_id
        req.marker_id = marker_id
        future = self.request_marker_client.call_async(req)
        
        if handle_response:
            future.add_done_callback(
                lambda f, mid=marker_id: self._marker_lock_response_callback(f, mid)
            )
    
    def _marker_lock_response_callback(self, future, marker_id: int):
        """Handle response from marker reservation request."""
        try:
            prev_id = self.locked_on_marker_id

            response = future.result()
            if response.success:
                self.locked_on_marker_id = marker_id
                self.logger.info(
                    f"Successfully locked on to marker {self.locked_on_marker_id}."
                )

                if prev_id != -1 and prev_id != marker_id:
                    # Unreserve previous marker if switching
                    self._unreserve_marker_id(prev_id)

                self.on_marker_locked(marker_id)
            else:
                self.logger.info(
                    f"Marker {marker_id} is not available: {response.message}"
                )
                self.on_marker_lost()
        except Exception as e:
            self.logger.error(f'Marker lock service call failed: {e}')
            self.on_marker_lost()
    
    def unreserve_current_marker(self):
        """Release the reservation on the currently locked marker."""
        if self.locked_on_marker_id < 0:
            return
        
        req = ReserveMarker.Request()
        req.drone_id = self.drone_id
        req.marker_id = self.locked_on_marker_id
        future = self.unreserve_marker_client.call_async(req)
        self.logger.info(f"Unreserved marker {self.locked_on_marker_id}")

    def _unreserve_marker_id(self, marker_id: int):
        """Release the reservation on a specific marker ID."""
        if marker_id < 0:
            return
        
        req = ReserveMarker.Request()
        req.drone_id = self.drone_id
        req.marker_id = marker_id
        future = self.unreserve_marker_client.call_async(req)
        self.logger.info(f"Unreserved marker {marker_id}")
    
    def renew_marker_reservation(self):
        """Renew reservation for the currently locked marker."""
        if self.locked_on_marker_id < 0:
            return
        
        # Silent renewal - no response handling needed
        self.request_marker_lock(self.locked_on_marker_id, handle_response=False)
    
    def mark_current_marker_landed(self):
        """Mark the currently locked marker as successfully landed."""
        if self.locked_on_marker_id < 0:
            return
        
        req = MarkLanded.Request()
        req.drone_id = self.drone_id
        req.marker_id = self.locked_on_marker_id
        future = self.mark_landed_client.call_async(req)
        
        self.logger.info(f"Marked marker {self.locked_on_marker_id} as landed")
    
    def reset_marker_state(self):
        """Reset all marker tracking state."""
        self.locked_on_marker_id = -1
        self.locked_on_marker_pose = None
        self.marker_last_seen_time = self.node.get_clock().now()
    
    def get_locked_marker_id(self) -> int:
        """Get the currently locked marker ID (-1 if none)."""
        return self.locked_on_marker_id
    
    def get_locked_marker_pose(self) -> Optional[Pose]:
        """Get the latest pose of the locked marker (None if not visible)."""
        return self.locked_on_marker_pose
    
    def is_marker_locked(self) -> bool:
        """Check if a marker is currently locked."""
        return self.locked_on_marker_id >= 0
    
    def is_marker_visible(self) -> bool:
        """Check if the locked marker is currently visible."""
        return self.locked_on_marker_pose is not None
