#!/usr/bin/env python3
"""
Waypoint Coordinator Module

Manages waypoint reservation and coordination with the WaypointServer
for multi-drone swarm operations. Prevents collisions by ensuring only
one drone navigates to each waypoint at a time.
"""

from typing import Optional, Set, Callable
from threading import Lock

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Int32MultiArray
from swarm_interfaces.msg import WaypointHeartbeat
from swarm_interfaces.srv import ReserveWaypoint


class WaypointCoordinator:
    """
    Manages waypoint reservation lifecycle for swarm coordination.
    
    Responsibilities:
    - Request waypoint reservations via async service calls
    - Track currently reserved waypoint index
    - Publish periodic heartbeats to maintain reservation
    - Monitor unavailable waypoints from swarm coordinator
    - Handle reservation failures and retries
    """
    
    __slots__ = (
        'node', 'logger', 'drone_id', 'reserved_waypoint_index',
        '_pending_waypoint_index', '_pending_callback',
        'unavailable_waypoints', '_unavailable_lock',
        'reserve_client', 'unreserve_client',
        'heartbeat_pub', 'unavailable_sub',
        '_coordination_enabled', '_reservation_in_progress'
    )
    
    def __init__(self, node: Node, drone_id: str):
        """
        Initialize waypoint coordinator.
        
        Args:
            node: ROS2 node for communication and logging
            drone_id: Unique identifier for this drone
        """
        self.node = node
        self.logger = node.get_logger()
        self.drone_id = drone_id
        
        # Reservation state tracking
        self.reserved_waypoint_index: int = -1  # Currently reserved waypoint
        self._pending_waypoint_index: int = -1  # Awaiting server response
        self._pending_callback: Optional[Callable] = None
        self._reservation_in_progress: bool = False
        
        # Unavailable waypoints tracking (thread-safe)
        self._unavailable_lock = Lock()
        self.unavailable_waypoints: Set[int] = set()
        
        # Coordination enable flag (fallback if server unavailable)
        self._coordination_enabled = True
        
        # Setup communication
        self._setup_communication()
        
        self.logger.info(f"WaypointCoordinator initialized for drone '{self.drone_id}'")
    
    def _setup_communication(self):
        """Setup ROS2 publishers, subscribers, and service clients."""
        # Heartbeat publisher (reliable QoS)
        heartbeat_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.heartbeat_pub = self.node.create_publisher(
            WaypointHeartbeat,
            '/waypoint_heartbeat',
            heartbeat_qos
        )
        
        # Subscribe to unavailable waypoints from server
        self.unavailable_sub = self.node.create_subscription(
            Int32MultiArray,
            '/unavailable_waypoints',
            self._unavailable_waypoints_callback,
            10
        )
        
        # Service clients for waypoint coordination
        self.reserve_client = self.node.create_client(
            ReserveWaypoint, '/reserve_waypoint'
        )
        
        self.unreserve_client = self.node.create_client(
            ReserveWaypoint, '/unreserve_waypoint'
        )
        
        # Wait for services with timeout
        if not self.reserve_client.wait_for_service(timeout_sec=5.0):
            self.logger.error(
                "WaypointServer (/reserve_waypoint) not available after 5s. "
                "Waypoint coordination DISABLED - operating in fallback mode!"
            )
            self._coordination_enabled = False
            return
        
        if not self.unreserve_client.wait_for_service(timeout_sec=5.0):
            self.logger.error(
                "WaypointServer (/unreserve_waypoint) not available after 5s. "
                "Waypoint coordination DISABLED - operating in fallback mode!"
            )
            self._coordination_enabled = False
            return
        
        self.logger.info("WaypointCoordinator: Connected to WaypointServer services")
    
    def _unavailable_waypoints_callback(self, msg: Int32MultiArray):
        """
        Update the list of unavailable waypoints from server.
        
        Thread-safe callback for subscriber running in executor thread.
        """
        with self._unavailable_lock:
            self.unavailable_waypoints = set(msg.data)
            
        self.logger.debug(
            f"Unavailable waypoints updated: {sorted(self.unavailable_waypoints)}",
            throttle_duration_sec=5.0
        )
    
    # ========================================================================
    # RESERVATION MANAGEMENT
    # ========================================================================
    
    def reserve_waypoint_async(
        self,
        waypoint_index: int,
        callback: Callable[[bool, str], None]
    ):
        """
        Request waypoint reservation via async service call (non-blocking).
        
        Args:
            waypoint_index: Index of waypoint to reserve
            callback: Function called with (success: bool, message: str) when complete
        """
        if not self._coordination_enabled:
            # Fallback mode - always succeed
            self.logger.warning(
                f"Coordination disabled - granting waypoint {waypoint_index} without server check",
                throttle_duration_sec=5.0
            )
            self.reserved_waypoint_index = waypoint_index
            callback(True, "Coordination disabled (fallback mode)")
            return
        
        if self._reservation_in_progress:
            self.logger.warning(
                f"Reservation already in progress for waypoint {self._pending_waypoint_index}. "
                f"Ignoring request for waypoint {waypoint_index}."
            )
            return
        
        # Set pending state
        self._pending_waypoint_index = waypoint_index
        self._pending_callback = callback
        self._reservation_in_progress = True
        
        # Make async service call
        req = ReserveWaypoint.Request()
        req.drone_id = self.drone_id
        req.waypoint_index = waypoint_index
        
        future = self.reserve_client.call_async(req)
        future.add_done_callback(
            lambda f: self._handle_reservation_response(f, waypoint_index)
        )
        
        self.logger.info(f"Requesting reservation for waypoint {waypoint_index}...")
    
    def _handle_reservation_response(self, future, waypoint_index: int):
        """
        Handle async response from waypoint reservation service.
        
        Called by ROS2 executor when service response arrives.
        """
        try:
            response = future.result()
            
            # Clear pending state
            self._reservation_in_progress = False
            pending_callback = self._pending_callback
            self._pending_callback = None
            self._pending_waypoint_index = -1
            
            if response.success:
                # SUCCESS: Reservation granted
                self.reserved_waypoint_index = waypoint_index
                self.logger.info(
                    f"✓ Waypoint {waypoint_index} reserved successfully"
                )
                
                if pending_callback:
                    pending_callback(True, response.message)
            else:
                # FAILURE: Waypoint already reserved by another drone
                self.logger.warning(
                    f"✗ Waypoint {waypoint_index} reservation failed: {response.message}"
                )
                
                if pending_callback:
                    pending_callback(False, response.message)
                    
        except Exception as e:
            # EXCEPTION: Service call failed
            self.logger.error(
                f"Waypoint reservation service call failed: {e}"
            )
            self._reservation_in_progress = False
            
            if self._pending_callback:
                self._pending_callback(False, f"Service exception: {e}")
            
            self._pending_callback = None
            self._pending_waypoint_index = -1
    
    def unreserve_waypoint(self, waypoint_index: int):
        """
        Release reservation for a specific waypoint (async, fire-and-forget).
        
        Args:
            waypoint_index: Index of waypoint to unreserve
        """
        if not self._coordination_enabled:
            return
        
        if waypoint_index < 0:
            return
        
        req = ReserveWaypoint.Request()
        req.drone_id = self.drone_id
        req.waypoint_index = waypoint_index
        
        future = self.unreserve_client.call_async(req)
        self.logger.info(f"Unreserved waypoint {waypoint_index}")
        
        # Clear reservation if it was the current one
        if self.reserved_waypoint_index == waypoint_index:
            self.reserved_waypoint_index = -1
    
    def unreserve_current_waypoint(self):
        """Release reservation for the currently reserved waypoint."""
        if self.reserved_waypoint_index >= 0:
            self.unreserve_waypoint(self.reserved_waypoint_index)
    
    def renew_waypoint_reservation(self):
        """
        Publish heartbeat to renew reservation for current waypoint.
        
        Call this periodically (e.g., 2 Hz) while navigating to waypoint.
        """
        if not self._coordination_enabled:
            return
        
        if self.reserved_waypoint_index < 0:
            return
        
        msg = WaypointHeartbeat()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.drone_id = self.drone_id
        msg.waypoint_index = self.reserved_waypoint_index
        
        self.heartbeat_pub.publish(msg)
        
        self.logger.debug(
            f"Heartbeat sent for waypoint {self.reserved_waypoint_index}",
            throttle_duration_sec=5.0
        )
    
    # ========================================================================
    # QUERY METHODS
    # ========================================================================
    
    def is_waypoint_available(self, waypoint_index: int) -> bool:
        """
        Check if a waypoint is available (not reserved by another drone).
        
        Args:
            waypoint_index: Index to check
            
        Returns:
            True if available, False if reserved by another drone
        """
        if not self._coordination_enabled:
            return True  # Always available in fallback mode
        
        with self._unavailable_lock:
            return waypoint_index not in self.unavailable_waypoints
    
    def is_waypoint_reserved_by_this_drone(self, waypoint_index: int) -> bool:
        """
        Check if this drone has reserved the specified waypoint.
        
        Args:
            waypoint_index: Index to check
            
        Returns:
            True if reserved by this drone
        """
        return self.reserved_waypoint_index == waypoint_index
    
    def is_reservation_pending(self) -> bool:
        """Check if a reservation request is awaiting response."""
        return self._reservation_in_progress
    
    def get_unavailable_waypoints_copy(self) -> Set[int]:
        """Get thread-safe copy of unavailable waypoints set."""
        with self._unavailable_lock:
            return self.unavailable_waypoints.copy()
    
    def is_coordination_enabled(self) -> bool:
        """Check if waypoint coordination is active."""
        return self._coordination_enabled
    
    # ========================================================================
    # CLEANUP
    # ========================================================================
    
    def reset(self):
        """Reset coordinator state (e.g., for mission restart)."""
        # Unreserve current waypoint
        self.unreserve_current_waypoint()
        
        # Clear all state
        self.reserved_waypoint_index = -1
        self._pending_waypoint_index = -1
        self._pending_callback = None
        self._reservation_in_progress = False
        
        with self._unavailable_lock:
            self.unavailable_waypoints.clear()
        
        self.logger.info("WaypointCoordinator reset")
