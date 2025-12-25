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
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from aruco_opencv_msgs.msg import ArucoDetection
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger

from .state_machine import MissionState, MissionManager
from .controller import DroneInterface
from .utils import ParameterLoader
from .aruco import ArucoMarkerHandler, ExitMarkerHandler, WaypointManager


class MissionControl(Node):
    """
    The central orchestrator node for the Tello drone's autonomous mission.
    """
    
    def __init__(self):
        super().__init__('mission_control')
        
        # Load parameters
        self.params = ParameterLoader(self)
        
        # Initialize drone interface
        self.drone = DroneInterface(
            self,
            self._on_action_success,
            self._on_action_fail
        )
        
        # Initialize takeoff and landing service servers
        self._setup_takeoff_server()
        self._setup_landing_server()
        
        # Initialize ArUco subscriber
        self._setup_aruco_sub()
        
        # Initialize ArUco marker handler
        self.marker_handler = ArucoMarkerHandler(
            node=self,
            drone_id=self.params.drone_id,
            priority_markers=self.params.priority_markers,
            on_marker_locked_callback=self._on_marker_locked,
            on_marker_lost_callback=self._on_marker_lost
        )

        # Initialize Exit Marker Handler
        self.exit_marker_handler = ExitMarkerHandler(
            priority_markers=set(self.params.priority_markers),
            exit_markers=set(self.params.exit_markers),
        )
        
        # Initialize waypoint manager
        if self.params.enable_waypoint_navigation:
            self.waypoint_manager = WaypointManager(
                node=self,
                waypoint_sequence=self.params.waypoint_sequence
            )
            self.get_logger().info(
                f"Waypoint navigation enabled with {self.waypoint_manager.get_waypoint_count()} waypoints"
            )
        else:
            self.waypoint_manager = None
            self.get_logger().info("Waypoint navigation disabled")
        
        # Initialize mission manager with all dependencies
        self.mission_manager = MissionManager(
            self,
            self.drone,
            self.marker_handler,
            self.waypoint_manager,
            self.params
        )
        
        # Current mission state
        self.mission_state = MissionState.IDLE
        state_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.state_pub = self.create_publisher(UInt8, 'mission_state', state_qos)
        self._last_published_state = None
        
        # Start main control loops
        self.main_timer = self.create_timer(0.25, self._main_logic_loop)  # 4 Hz
        self.reservation_timer = self.create_timer(2.0, self._renew_marker_reservation)
        
        self.get_logger().info(
            f"Mission Control Node started! Current state: {self.mission_state.name}"
        )
    
    def _setup_takeoff_server(self):
        """Initialize takeoff service server."""
        self.takeoff_srv = self.create_service(
            Trigger,
            'takeoff',
            self._handle_takeoff_request
        )

    def _setup_landing_server(self):
        """Initialize landing service server."""
        self.landing_srv = self.create_service(
            Trigger,
            'land',
            self._handle_landing_request
        )
        
    def _setup_aruco_sub(self):
        """Initialize ArUco subscriber."""
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # ArUco subscriber (complex state-dependent logic)
        self.aruco_sub = self.create_subscription(
            ArucoDetection,
            'aruco_detections',
            self._aruco_callback,
            qos_profile
        )
        
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
        # Scenario 0: Update waypoint marker pose during waypoint navigation
        if self.mission_state in [
            MissionState.WAYPOINT_CENTERING,
            MissionState.WAYPOINT_APPROACHING
        ]:
            if self.waypoint_manager:
                self.waypoint_manager.update_marker_pose(msg)
            return
        
        # Scenario 1: Searching and found a marker
        if self.mission_state == MissionState.SEARCHING and len(msg.markers) > 0:
            if self.marker_handler.process_aruco_detection_for_search(msg):
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
    
    def _renew_marker_reservation(self):
        """Periodically renew marker reservation during approach/landing."""
        if self.mission_state in [
            MissionState.CENTERING,
            MissionState.APPROACHING,
            MissionState.CAMERA_SWITCHING,
            MissionState.PRECISION_LANDING,
            MissionState.LANDING
        ]:
            self.marker_handler.renew_marker_reservation()
    
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
        
        # Check for action timeouts
        self.drone.check_timeout()
        
        # Wait until current action completes
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
    node = MissionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
