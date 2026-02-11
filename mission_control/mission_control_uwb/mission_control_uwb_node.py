#!/usr/bin/env python3
"""
Thin orchestrator node for mission control system.

This node initializes subsystems and coordinates the autonomous drone mission.
The heavy lifting is delegated to specialized modules:
- MissionManager: State execution logic
- DroneInterface: Control commands and telemetry
- ArucoMarkerHandler: Marker selection and swarm coordination
- ParameterLoader: Configuration management
"""
import os

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from aruco_opencv_msgs.msg import ArucoDetection
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger

from .state_machine import MissionState, MissionManager
from .controller import DroneInterface
from .utils import ParameterLoader
from .aruco import ArucoMarkerHandler, ExitMarkerHandler
from .uwb import WaypointManager, UWBNavigator, WaypointCoordinator


class MissionControlUWB(Node):
    """
    The central orchestrator node for the Tello drone's autonomous mission.
    """
    
    def __init__(self):
        super().__init__('mission_control_uwb')
        
        self.params = ParameterLoader(self)
        
        self._init_drone_interface()
        self._init_services()
        self._init_subscriptions()
        self._init_marker_handlers()
        self._init_waypoint_navigation()
        self._init_mission_manager()
        self._init_state_publisher()
        self._init_timers()
        
        self.get_logger().info(
            f"Mission Control Node started! Current state: {self.mission_state.name}"
        )
    
    # ========================================================================
    # INITIALIZATION HELPERS
    # ========================================================================
    
    def _init_drone_interface(self):
        """Initialize drone interface for control and telemetry."""
        self.drone = DroneInterface(
            self,
            self._on_action_success,
            self._on_action_fail
        )
    
    def _init_services(self):
        """Initialize ROS2 service servers."""
        self.takeoff_srv = self.create_service(
            Trigger, 'takeoff', self._handle_takeoff_request
        )
        self.landing_srv = self.create_service(
            Trigger, 'land', self._handle_landing_request
        )
    
    def _init_subscriptions(self):
        """Initialize ROS2 subscriptions."""
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.aruco_sub = self.create_subscription(
            ArucoDetection, 'aruco_detections',
            self._aruco_callback, qos_best_effort
        )
    
    def _init_marker_handlers(self):
        """Initialize ArUco marker and exit marker handlers."""
        self.marker_handler = ArucoMarkerHandler(
            node=self,
            drone_id=self.params.drone_id,
            priority_markers=self.params.priority_markers,
            on_marker_locked_callback=self._on_marker_locked,
            on_marker_lost_callback=self._on_marker_lost
        )
        self.exit_marker_handler = ExitMarkerHandler(
            priority_markers=set(self.params.priority_markers),
            exit_markers=set(self.params.exit_markers),
        )
    
    def _init_waypoint_navigation(self):
        """Initialize waypoint manager and UWB navigator."""
        if not self.params.waypoints_file:
            self.get_logger().error(
                "Waypoint navigation enabled but 'waypoints_file' not specified"
            )
            self.waypoint_manager = None
            self.uwb_navigator = None
            self.waypoint_coordinator = None
            return
        
        waypoints_path = self._resolve_waypoints_path(self.params.waypoints_file)
        
        try:
            self.waypoint_manager = WaypointManager(
                node=self,
                waypoints_file=waypoints_path
            )
        except (FileNotFoundError, ValueError) as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            self.waypoint_manager = None
            self.uwb_navigator = None
            self.waypoint_coordinator = None
            return
        
        # Initialize waypoint coordinator for swarm coordination
        self.waypoint_coordinator = WaypointCoordinator(
            node=self,
            drone_id=self.params.drone_id
        )
        
        self.uwb_navigator = UWBNavigator(
            drone_interface=self.drone,
            node=self,
            tolerance_xy=self.params.waypoint_tolerance_xy,
            tolerance_yaw=self.params.waypoint_tolerance_yaw,
            forward_speed=self.params.waypoint_forward_speed,
            strafe_speed=self.params.waypoint_strafe_speed,
            yaw_speed=self.params.waypoint_yaw_speed
        )
        self.get_logger().info("UWB Navigator and WaypointCoordinator initialized")
    
    def _resolve_waypoints_path(self, waypoints_file: str) -> str:
        """
        Resolve waypoints file path.
        
        If the path is relative (no leading '/'), resolve it relative to
        tello_bringup/missions directory. Absolute paths are returned as-is.
        
        Args:
            waypoints_file: Filename or absolute path to waypoints JSON
            
        Returns:
            Absolute path to waypoints file
        """
        if os.path.isabs(waypoints_file):
            return waypoints_file
        
        # Resolve relative path to tello_bringup/missions/
        missions_dir = os.path.join(
            get_package_share_directory('tello_bringup'),
            'missions'
        )
        resolved_path = os.path.join(missions_dir, waypoints_file)
        self.get_logger().debug(f"Resolved waypoints path: {resolved_path}")
        return resolved_path
    
    def _init_mission_manager(self):
        """Initialize mission manager with all dependencies."""
        self.mission_manager = MissionManager(
            self,
            self.drone,
            self.marker_handler,
            self.waypoint_manager,
            self.uwb_navigator,
            self.waypoint_coordinator,
            self.params
        )
    
    def _init_state_publisher(self):
        """Initialize mission state tracking and publisher."""
        self.mission_state = MissionState.IDLE
        self._last_published_state = None
        
        state_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.state_pub = self.create_publisher(UInt8, 'mission_state', state_qos)
    
    def _init_timers(self):
        """Initialize control loop timers."""
        self.main_timer = self.create_timer(0.25, self._main_logic_loop)  # 4 Hz
        self.reservation_timer = self.create_timer(2.0, self._renew_reservations)
        
    def _publish_mission_state(self):
        """Publish mission state transitions for monitoring dashboards."""
        current_value = self.mission_state.value
        if current_value == self._last_published_state:
            return
        msg = UInt8()
        msg.data = current_value
        self.state_pub.publish(msg)
        self._last_published_state = current_value

    # ========================================================================
    # ROS2 CALLBACK METHODS
    # ========================================================================
    
    def _aruco_callback(self, msg: ArucoDetection):
        """Process ArUco detections and coordinate with marker handler."""
        # Scenario 1: Searching and found a marker
        if self.mission_state == MissionState.SEARCHING and len(msg.markers) > 0:
            if self.marker_handler.process_aruco_detection_for_search(msg):
                # Cancel any ongoing yaw rotation to keep marker in view
                self.drone.cancel_yaw()
                self.mission_state = MissionState.LOCKING_ON
                return
            
            # Check for exit marker proximity
            if self.exit_marker_handler.is_near_exit_marker(msg):
                self.mission_manager.is_near_exit = True
            else:
                self.mission_manager.is_near_exit = False
            return
        
        # Scenario 2: Centering and may need to switch to priority marker
        if self.mission_state in [MissionState.CENTERING]:
            if self.marker_handler.should_switch_to_priority_marker(msg):
                if self.marker_handler.process_aruco_detection_for_search(msg):
                    self.mission_state = MissionState.LOCKING_ON
                    return
        
        # Scenario 2.5: Priority scanning - detect priority markers during 360° scan
        if self.mission_state == MissionState.PRIORITY_SCANNING and len(msg.markers) > 0:
            if self.marker_handler.should_switch_to_priority_marker(msg):
                if self.marker_handler.process_aruco_detection_for_search(msg):
                    self.drone.cancel_yaw()  # Cancel ongoing rotation
                    self.get_logger().info(
                        "PRIORITY_SCANNING: Priority marker detected! Switching target."
                    )
                    self.mission_state = MissionState.LOCKING_ON
                    return
        
        # Scenario 3: Update locked marker pose
        if self.mission_state in [
            MissionState.CENTERING,
            MissionState.APPROACHING,
            MissionState.PRECISION_LANDING
        ]:
            self.marker_handler.update_locked_marker_pose(msg)
    
    def _handle_takeoff_request(self, request, response):
        """Handle takeoff service request."""
        if self.mission_state != MissionState.IDLE:
            response.success = False
            response.message = (
                f"Cannot takeoff: Current state is {self.mission_state.name}, not IDLE."
            )
            self.get_logger().warning(response.message)
            return response
        
        self.get_logger().info("Takeoff requested. Transitioning to TAKING_OFF state.")
        self.mission_state = MissionState.TAKING_OFF
        response.success = True
        response.message = "Takeoff initiated."
        return response
    
    def _handle_landing_request(self, request, response):
        """Handle landing service request."""
        if self.mission_state in [
            MissionState.LANDING,
            MissionState.COMPLETING_MISSION,
            MissionState.RESETTING,
            MissionState.IDLE
        ]:
            response.success = False
            response.message = "Landing already in progress."
            self.get_logger().warning(response.message)
            return response
        
        self.get_logger().info("Landing requested. Transitioning to LANDING state.")
        self.mission_state = MissionState.LANDING
        response.success = True
        response.message = "Landing initiated."
        return response
    
    def _renew_reservations(self):
        """Periodically renew both marker and waypoint reservations (2 Hz)."""
        # Renew marker reservation during approach/landing
        if self.mission_state in [
            MissionState.CENTERING,
            MissionState.APPROACHING,
            MissionState.CAMERA_SWITCHING,
            MissionState.PRIORITY_SCANNING,
            MissionState.PRECISION_LANDING,
            MissionState.LANDING
        ]:
            self.marker_handler.renew_marker_reservation()
        
        # Renew waypoint reservation during navigation
        if self.mission_state in [
            MissionState.WAYPOINT_NAVIGATION,
            MissionState.SEARCHING
        ]:
            if self.waypoint_coordinator is not None:
                self.waypoint_coordinator.renew_waypoint_reservation()
    
    # ========================================================================
    # ACTION MANAGER CALLBACKS
    # ========================================================================
    
    def _on_action_success(self, next_state: MissionState):
        """Callback for successful action completion."""
        if next_state is not None:
            self.get_logger().info(
                f"Action succeeded! Transitioning to {next_state.name}."
            )
            self.mission_state = next_state
    
    def _on_action_fail(self, fallback_state: MissionState = MissionState.SEARCHING):
        """Callback for action failure or timeout."""
        if fallback_state is None:
            fallback_state = MissionState.SEARCHING
        self.get_logger().error(f"Action failed! Fallback to {fallback_state.name}.")
        self.mission_state = fallback_state
    
    # ========================================================================
    # MARKER HANDLER CALLBACKS
    # ========================================================================
    
    def _on_marker_locked(self, marker_id: int):
        """Callback when marker successfully locked."""
        self.get_logger().info(
            f"Marker {marker_id} locked. Transitioning to CENTERING."
        )
        self.mission_state = MissionState.CENTERING
    
    def _on_marker_lost(self):
        """Callback when marker lock fails or is lost."""
        self.get_logger().info("Marker lock failed. Resuming SEARCHING.")
        self.mission_state = MissionState.SEARCHING
    
    # ========================================================================
    # MAIN CONTROL LOOP
    # ========================================================================
    
    def _main_logic_loop(self):
        """
        Main state machine control loop (4 Hz).
        
        Delegates state execution to the MissionManager.
        """
        self._publish_mission_state()
        
        # Check for action timeouts and yaw rotation progress
        self.drone.check_timeout()
        self.drone.check_yaw_progress()
        
        # Wait until current action or yaw rotation completes
        if self.drone.is_busy():
            return
        
        # Special case: LOCKING_ON - hover while waiting for marker server
        if self.mission_state == MissionState.LOCKING_ON:
            # self.drone.hover()
            return
        
        # Special case: IDLE - do nothing
        if self.mission_state == MissionState.IDLE:
            return
        
        # Execute current state logic
        new_state = self.mission_manager.execute_state(self.mission_state)
        if new_state is not None:
            self.mission_state = new_state


def main(args=None):
    """Main entry point for mission control node."""
    rclpy.init(args=args)
    node = MissionControlUWB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
