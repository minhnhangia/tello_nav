#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from enum import Enum
import time
import math
import numpy as np

# Import ROS2 message and service types
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from midas_msgs.msg import DepthMapAnalysis
from tello_msgs.msg import FlightData, TelloResponse
from aruco_opencv_msgs.msg import ArucoDetection
from tello_msgs.srv import TelloAction
from std_srvs.srv import Trigger

from .action_manager import ActionManager
from .aruco_marker_handler import ArucoMarkerHandler

class MissionState(Enum):
    """Defines the operational states of the drone."""
    IDLE = 0
    TAKING_OFF = 1
    SEARCHING = 2
    LOCKING_ON = 3
    CENTERING = 4
    APPROACHING = 5
    CAMERA_SWITCHING = 6
    PRECISION_LANDING = 7
    LANDING = 8
    COMPLETING_MISSION = 9

class MissionControl(Node):
    """
    The central 'brain' node for the Tello drone's autonomous mission.
    """
    def __init__(self):
        super().__init__('mission_control')

        # === Initialization ===
        self.init_mission_state()
        self.set_all_params()
        self.set_pubs_subs()
        self.set_service_clients()
        self.set_service_servers()

        # Instantiate the ActionManager
        self.action_manager = ActionManager(self, self.action_client, 
                                            'tello_response',
                                            self.on_action_success, 
                                            self.on_action_fail,
                                            self.invalidate_sensor_data)

        # Instantiate the ArUco Marker Handler
        self.marker_handler = ArucoMarkerHandler(
            node=self,
            drone_id=self.DRONE_ID,
            priority_markers=self.PRIORITY_MARKERS,
            on_marker_locked_callback=self.on_marker_locked,
            on_marker_lost_callback=self.on_marker_lost
        )

        # === Main Logic Timer ===
        self.timer = self.create_timer(0.25, self.main_logic_loop) # 4 Hz loop
        self.renew_reservation_timer = self.create_timer(5.0, self.renew_marker_reservation)
        self.get_logger().info(f"Mission Control Node has started! Current state: {self.mission_state.name}")

    def init_mission_state(self):
        self.mission_state = MissionState.IDLE  # Initial state
        self.state_handlers = {
            MissionState.TAKING_OFF: self.run_taking_off_logic,
            MissionState.SEARCHING: self.run_searching_logic,
            MissionState.CENTERING: self.run_centering_logic,
            MissionState.APPROACHING: self.run_approaching_logic,
            MissionState.CAMERA_SWITCHING: self.run_camera_switching_logic,
            MissionState.PRECISION_LANDING: self.run_precision_landing_logic,
            MissionState.LANDING: self.run_landing_logic,
            MissionState.COMPLETING_MISSION: self.run_completing_mission_logic,
        }

    def set_all_params(self):
        # === Parameters ===
        self.declare_parameter('drone_id', 'tello1')
        self.declare_parameter('priority_markers', []) # high-value targets to preferentially seek
        self.declare_parameter('min_takeoff_height', 0.3)
        # SEARCHING state parameters
        self.declare_parameter('yaw_speed', 0.5)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('sideway_speed', 0.1)
        self.declare_parameter('corner_tof_threshold', 0.9)
        self.declare_parameter('headon_tof_threshold', 0.5)
        # self.declare_parameter('altitude_check_interval_s', 120.0)
        # self.declare_parameter('altitude_lower_step', 0.20)
        # self.declare_parameter('initial_search_height', 1.3)
        # CENTERING state parameters
        self.declare_parameter('centering_threshold_x', 0.12)
        self.declare_parameter('centering_yaw_kp', 0.3) # Proportional gain for yaw control
        self.declare_parameter('marker_timeout_s', 2.0) # Timeout for marker detection
        # APPROACHING state parameters
        self.declare_parameter('max_approach_dist', 5.0)
        self.declare_parameter('step_approach_dist', 1.0)
        self.declare_parameter('final_approach_offset', 0.3)
        # PRECISION_LANDING parameters
        self.declare_parameter('precision_landing_threshold_x', 0.15)
        self.declare_parameter('precision_landing_threshold_y', 0.15)
        self.declare_parameter('precision_landing_max_speed', 0.2)
        self.declare_parameter('precision_sideway_kp', 0.6)
        self.declare_parameter('precision_forward_kp', 0.6)
        self.declare_parameter('precision_landing_timeout_s', 10.0)

        # Assign parameters to member variables
        self.DRONE_ID = self.get_parameter('drone_id').get_parameter_value().string_value
        self.PRIORITY_MARKERS = set(self.get_parameter('priority_markers').value)
        self.MIN_TAKEOFF_HEIGHT = self.get_parameter('min_takeoff_height').value
        self.YAW_SPEED = self.get_parameter('yaw_speed').value
        self.FORWARD_SPEED = self.get_parameter('forward_speed').value
        self.SIDEWAY_SPEED = self.get_parameter('sideway_speed').value
        self.CORNER_TOF_THRESHOLD = self.get_parameter('corner_tof_threshold').value
        self.HEADON_TOF_THRESHOLD = self.get_parameter('headon_tof_threshold').value
        # self.ALTITUDE_CHECK_INTERVAL = self.get_parameter('altitude_check_interval_s').value
        # self.ALTITUDE_LOWER_STEP = self.get_parameter('altitude_lower_step').value
        # self.INITIAL_SEARCH_HEIGHT = self.get_parameter('initial_search_height').value
        self.CENTERING_THRESHOLD_X = self.get_parameter('centering_threshold_x').value
        self.CENTERING_YAW_KP = self.get_parameter('centering_yaw_kp').value
        self.MARKER_TIMEOUT = self.get_parameter('marker_timeout_s').value
        self.MAX_APPROACH_DIST = self.get_parameter('max_approach_dist').value
        self.STEP_APPROACH_DIST = self.get_parameter('step_approach_dist').value
        self.FINAL_APPROACH_OFFSET = self.get_parameter('final_approach_offset').value
        self.PRECISION_LANDING_THRESHOLD_X = self.get_parameter('precision_landing_threshold_x').value
        self.PRECISION_LANDING_THRESHOLD_Y = self.get_parameter('precision_landing_threshold_y').value
        self.PRECISION_LANDING_MAX_SPEED = self.get_parameter('precision_landing_max_speed').value
        self.PRECISION_SIDEWAY_KP = self.get_parameter('precision_sideway_kp').value
        self.PRECISION_FORWARD_KP = self.get_parameter('precision_forward_kp').value
        self.PRECISION_LANDING_TIMEOUT = self.get_parameter('precision_landing_timeout_s').value

        # === State & Sensor Data Variables ===
        self.latest_tof = None
        self.latest_depth_analysis = None
        self.latest_height = 0.0
        self.latest_battery = 100
        self.time_search_started = self.get_clock().now()

    def set_pubs_subs(self):
        # === ROS2 Communications ===
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.tof_sub = self.create_subscription(Range, 'ext_tof', self.tof_callback, qos_profile)
        self.depth_sub = self.create_subscription(DepthMapAnalysis, 'depth/analysis', self.depth_analysis_callback, qos_profile)
        self.flight_data_sub = self.create_subscription(FlightData, 'flight_data', self.flight_data_callback, qos_profile)
        self.aruco_sub = self.create_subscription(ArucoDetection, 'aruco_detections', self.aruco_callback, qos_profile)

    def set_service_clients(self):
        # Service Clients
        self.action_client = self.create_client(TelloAction, 'tello_action')
        while not self.action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Action service not available, waiting...')

    def set_service_servers(self):
        self.takeoff_srv = self.create_service(Trigger, 'takeoff', self.handle_takeoff_request)

    # --- Subscriber Callbacks ---
    def tof_callback(self, msg : Range):
        self.latest_tof = msg.range

    def depth_analysis_callback(self, msg : DepthMapAnalysis):
        self.latest_depth_analysis = msg

    def flight_data_callback(self, msg : FlightData):
        self.latest_height = float(msg.tof) / 100.0  # height measured by ToF in meters
        self.latest_battery = msg.bat  # battery percentage
        if self.latest_battery < 20:
            self.get_logger().error(f"CRITICAL: Battery at {self.latest_battery}%", throttle_duration_sec=5.0)
        elif self.latest_battery < 50:
            self.get_logger().warning(f"Battery at {self.latest_battery}%", throttle_duration_sec=15.0)

    # --- ActionManager callbacks ---
    def on_action_success(self, next_state : MissionState):
        """Callback for when an action succeeds."""
        if next_state is not None:
            self.get_logger().info(f"Action succeeded! Transitioning to {next_state.name}.")
            self.mission_state = next_state

    def on_action_fail(self, fallback_state : MissionState = MissionState.SEARCHING):
        """Callback for when an action fails, is rejected, or times out."""
        self.get_logger().error(f"Action failed! Fallback to {fallback_state.name}.")
        self.mission_state = fallback_state

    def invalidate_sensor_data(self):
        """
        Sets critical sensor data to None. This is called just before
        an action is executed to prevent acting on stale data post-action.
        """
        # self.get_logger().info("Executing action, invalidating last ToF and depth data.")
        self.latest_tof = None
        self.latest_depth_analysis = None

    # --- Marker Handler callbacks ---
    def on_marker_locked(self, marker_id: int):
        """Callback when marker successfully locked."""
        self.get_logger().info(f"Marker {marker_id} locked. Transitioning to CENTERING.")
        self.mission_state = MissionState.CENTERING
    
    def on_marker_lost(self):
        """Callback when marker lock fails or is lost."""
        self.get_logger().info("Marker lock failed. Resuming SEARCHING.")
        self.mission_state = MissionState.SEARCHING

    def aruco_callback(self, msg: ArucoDetection):
        """Process ArUco detections and delegate to marker handler."""
        # Scenario 1: We are searching and find a marker for the first time.
        if self.mission_state == MissionState.SEARCHING and len(msg.markers) > 0:
            if self.marker_handler.process_aruco_detection_for_search(msg):
                # Marker lock initiated, transition to LOCKING_ON
                self.mission_state = MissionState.LOCKING_ON
            
        # Scenario 2: We are already locked on and need to update the pose.
        elif self.mission_state in [MissionState.CENTERING, MissionState.APPROACHING, MissionState.PRECISION_LANDING]:
            self.marker_handler.update_locked_marker_pose(msg)

    # --- Service Callbacks ---
    def handle_takeoff_request(self, request, response):
        if self.mission_state != MissionState.IDLE:
            response.success = False
            response.message = f"Cannot take off: Current state is {self.mission_state.name}, not IDLE."
            self.get_logger().warning(response.message)
            return response

        self.get_logger().info("Takeoff requested. Transitioning to TAKING_OFF state.")
        self.mission_state = MissionState.TAKING_OFF
        response.success = True
        response.message = "Takeoff initiated."
        return response
    
    def renew_marker_reservation(self):
        """Periodically renew reservation for the currently targeted marker while approaching/landing."""
        if self.mission_state not in [MissionState.CENTERING, MissionState.APPROACHING, 
                                      MissionState.CAMERA_SWITCHING, MissionState.PRECISION_LANDING, 
                                      MissionState.LANDING]:
            return
        self.marker_handler.renew_marker_reservation()

    # --- Main Control Loop & State Logic ---
    def main_logic_loop(self):
        # State machine dispatcher

        self.action_manager.check_timeout()

        # Wait until current action completes
        if self.action_manager.is_busy():
            return  

        # If we are waiting for the marker server, HOVER.
        if self.mission_state == MissionState.LOCKING_ON:
            self.cmd_vel_pub.publish(Twist())
            return
        
        # If IDLE, do nothing
        if self.mission_state == MissionState.IDLE:
            return
        
        # Dispatch to the appropriate state handler
        handler = self.state_handlers.get(self.mission_state)
        if handler:
            handler()
        
    def run_taking_off_logic(self):
        if self.latest_height >= self.MIN_TAKEOFF_HEIGHT:
            self.get_logger().info(f"TAKING_OFF: Already airborne at {self.latest_height:.2f}m. Transitioning to SEARCHING.")
            self.mission_state = MissionState.SEARCHING
            return
        
        self.get_logger().info("TAKING_OFF: attempting takeoff.")
        self.action_manager.execute_action(f'takeoff', MissionState.SEARCHING, MissionState.TAKING_OFF)

    def run_searching_logic(self):
        # Wait until all necessary sensor data is available
        if self.latest_tof is None or self.latest_depth_analysis is None:
            self.get_logger().info('SEARCHING: Waiting for sensor data...', throttle_duration_sec=5)
            return

        # Check for altitude adjustment
        # self.check_and_lower_altitude()

        twist_msg = Twist()
        depth = self.latest_depth_analysis

        # 1. ToF Error Check
        if self.latest_tof is None:
            self.get_logger().info("SEARCHING: Waiting for EXT TOF data...", throttle_duration_sec=5)
            self.cmd_vel_pub.publish(twist_msg)
            return
        if self.latest_tof > 8.888:     # TO DO: confirm this is the correct out-of-range value
            self.get_logger().info("SEARCHING: ToF out of range or error. Hovering.")
            self.cmd_vel_pub.publish(twist_msg)
            return
        
        # 2. Obstacle Ahead (Depth Map)
        if depth.middle_center.red > depth.middle_center.blue:
            if depth.middle_left.blue > depth.middle_right.blue:
                self.get_logger().warning("SEARCHING: Obstacle detected. Turning Left.")
                twist_msg.angular.z = -self.YAW_SPEED
            else:
                self.get_logger().warning("SEARCHING: Obstacle detected. Turning Right.")
                twist_msg.angular.z = self.YAW_SPEED
            self.cmd_vel_pub.publish(twist_msg)
        
        # # 3. Corner Avoidance (Depth Clear + ToF Close)
        elif depth.middle_center.blue > depth.middle_center.red and self.latest_tof <= self.CORNER_TOF_THRESHOLD:
            self.get_logger().warning("SEARCHING: Corner detected. Executing 150-degree clockwise rotation.")
            self.action_manager.execute_action('cw 150', MissionState.SEARCHING, MissionState.SEARCHING)

        # # 4. Head-on Avoidance (ToF Very Close)
        elif self.latest_tof <= self.HEADON_TOF_THRESHOLD:
            self.get_logger().warning("SEARCHING: Head-on obstacle detected. Executing 180-degree rotation.")
            self.action_manager.execute_action('cw 180', MissionState.SEARCHING, MissionState.SEARCHING)

        # TO DO: add recovery behavior if stuck
        
        # 5. Path Clear
        else:
            self.get_logger().info("SEARCHING: Path clear. Moving forward.", throttle_duration_sec=2)
            twist_msg.linear.x = self.FORWARD_SPEED
            self.cmd_vel_pub.publish(twist_msg)

    def run_centering_logic(self):
        if self.latest_depth_analysis is None:
            self.get_logger().warning("CENTERING: Waiting for depth analysis data...")
            return
        
        # Check for marker timeout
        if self.marker_handler.check_marker_timeout(self.MARKER_TIMEOUT):
            self.get_logger().warning(f"CENTERING: Lost marker for >{self.MARKER_TIMEOUT}s. Returning to SEARCHING.")
            self.marker_handler.unreserve_current_marker()
            self.mission_state = MissionState.SEARCHING
            return

        # Check if marker is visible
        if not self.marker_handler.is_marker_visible():
            self.get_logger().warning("CENTERING: Marker temporarily lost. Hovering and waiting...", throttle_duration_sec=1.0)
            self.cmd_vel_pub.publish(Twist())
            return

        twist_msg = Twist()
        depth_analysis = self.latest_depth_analysis

        # --- 1. OBSTACLE AVOIDANCE (HIGHEST PRIORITY) ---
        # Check if an obstacle is blocking the left side of the path to the marker
        if depth_analysis.mc_left.nonblue - 100 > depth_analysis.mc_left.blue:
            self.get_logger().warning("CENTERING: Obstacle on the left, moving RIGHT.")
            twist_msg.linear.y = self.SIDEWAY_SPEED
            self.cmd_vel_pub.publish(twist_msg)
            return

        # Check if an obstacle is blocking the right side of the path
        elif depth_analysis.mc_right.nonblue - 100 > depth_analysis.mc_right.blue:
            self.get_logger().warning("CENTERING: Obstacle on the right, moving LEFT.")
            twist_msg.linear.y = -self.SIDEWAY_SPEED
            self.cmd_vel_pub.publish(twist_msg)
            return 
        
        # --- 2. YAW CENTERING (RUNS IF PATH IS CLEAR) ---
        marker_pose = self.marker_handler.get_locked_marker_pose()
        x_error = marker_pose.position.x
        
        if abs(x_error) > self.CENTERING_THRESHOLD_X:
            yaw_speed = np.clip(x_error * self.CENTERING_YAW_KP, -self.YAW_SPEED, self.YAW_SPEED)
            twist_msg.angular.z = yaw_speed
            self.get_logger().info(f"CENTERING: x_error: {x_error:.2f}m, yaw_speed: {yaw_speed:.2f}", throttle_duration_sec=1.0)
            self.cmd_vel_pub.publish(twist_msg)
        else:
            self.get_logger().info("CENTERING: Centered on marker. Transitioning to APPROACHING.")
            self.cmd_vel_pub.publish(Twist()) # Stop rotation
            self.mission_state = MissionState.APPROACHING
        
    def run_approaching_logic(self):
        # Check for marker timeout
        if self.marker_handler.check_marker_timeout(self.MARKER_TIMEOUT):
            self.get_logger().error(f"APPROACHING: Lost marker for >{self.MARKER_TIMEOUT}s. Returning to CENTERING.")
            self.mission_state = MissionState.CENTERING
            return

        # Check if marker is visible
        if not self.marker_handler.is_marker_visible():
            self.get_logger().warning("APPROACHING: Marker temporarily lost. Hovering and waiting...", throttle_duration_sec=1.0)
            self.cmd_vel_pub.publish(Twist())
            return
        
        marker_pose = self.marker_handler.get_locked_marker_pose()
        x = marker_pose.position.x
        y = marker_pose.position.y
        z = marker_pose.position.z

        dist_2d = math.sqrt(z**2 + x**2)
        forward_dist = z

        if forward_dist < self.MAX_APPROACH_DIST:
            self.get_logger().info(f"APPROACHING: moving forward {forward_dist:.2f} m, x = {x:.2f} m, y = {y:.2f} m")
            forward_dist_cmd = max(0, int(forward_dist * 100 - self.FINAL_APPROACH_OFFSET * 100))
            self.action_manager.execute_action(f'forward {forward_dist_cmd}', MissionState.CAMERA_SWITCHING, MissionState.CENTERING)
        else:
            self.get_logger().info(f"APPROACHING: Too far from marker ({forward_dist:.2f} m). Stepping forward {self.STEP_APPROACH_DIST} m.")
            forward_dist_cmd = int(self.STEP_APPROACH_DIST * 100)
            self.action_manager.execute_action(f'forward {forward_dist_cmd}', MissionState.CENTERING, MissionState.CENTERING)

    def run_camera_switching_logic(self):
        self.get_logger().info("Switching to downward-facing camera for precision landing.")
        self.action_manager.execute_action(f'downvision 1', MissionState.PRECISION_LANDING, MissionState.CAMERA_SWITCHING)

    def run_precision_landing_logic(self):
        # Check for marker timeout
        if self.marker_handler.check_marker_timeout(self.PRECISION_LANDING_TIMEOUT):
            self.get_logger().warning(f"PRECISION_LANDING: Lost marker for >{self.PRECISION_LANDING_TIMEOUT}s. Landing blind!")
            self.cmd_vel_pub.publish(Twist()) # Stop moving
            self.mission_state = MissionState.LANDING
            return

        # Check if marker is visible
        if not self.marker_handler.is_marker_visible():
            self.get_logger().warning("PRECISION_LANDING: Marker temporarily lost. Hovering and waiting...", throttle_duration_sec=1.0)
            self.cmd_vel_pub.publish(Twist())
            return

        twist_msg = Twist()
        
        # Extract positional errors (in meters)
        marker_pose = self.marker_handler.get_locked_marker_pose()
        y_error = marker_pose.position.y
        x_error = marker_pose.position.x

        # Check if we are centered on both axes
        x_centered = abs(x_error) <= self.PRECISION_LANDING_THRESHOLD_X
        y_centered = abs(y_error) <= self.PRECISION_LANDING_THRESHOLD_Y

        if x_centered and y_centered:
            self.get_logger().info("PRECISION_LANDING: Centered over marker. Transitioning to LANDING.")
            self.cmd_vel_pub.publish(Twist()) # Stop moving
            self.mission_state = MissionState.LANDING
            return

        # Proportional Control for final alignment
        if not x_centered:
            # if x_error is positive, need to move backward
            forward_speed = np.clip(-x_error * self.PRECISION_FORWARD_KP, -self.PRECISION_LANDING_MAX_SPEED, self.PRECISION_LANDING_MAX_SPEED)
            twist_msg.linear.x = forward_speed     
        
        if not y_centered:
            # if y_error is positive, need to move left
            sideway_speed = np.clip(-y_error * self.PRECISION_SIDEWAY_KP, -self.PRECISION_LANDING_MAX_SPEED, self.PRECISION_LANDING_MAX_SPEED)
            twist_msg.linear.y = sideway_speed     

        self.get_logger().info(f"PRECISION_LANDING: x_err: {x_error:.2f}, y_err: {y_error:.2f}, speed: {twist_msg.linear}", throttle_duration_sec=1.0)
        self.cmd_vel_pub.publish(twist_msg)

    def run_landing_logic(self):
        if self.latest_height < self.MIN_TAKEOFF_HEIGHT:
            self.get_logger().info(f"LANDING: Already landed at {self.latest_height:.2f}m. Transitioning to COMPLETING_MISSION.")
            self.mission_state = MissionState.COMPLETING_MISSION
            return
        
        self.action_manager.execute_action("land", MissionState.COMPLETING_MISSION, MissionState.LANDING)

    def run_completing_mission_logic(self):
        self.get_logger().info("COMPLETING_MISSION: Marking marker as landed and completing mission.")
        self.marker_handler.mark_current_marker_landed()
        self.mission_state = MissionState.IDLE  # Reset to IDLE after mission completion

    # def check_and_lower_altitude(self):
    #     """Checks if enough time has passed to lower the drone's search altitude."""
    #     elapsed_time = (self.get_clock().now() - self.time_search_started).nanoseconds / 1e9
        
    #     if elapsed_time > self.ALTITUDE_CHECK_INTERVAL and self.latest_height_cm >= self.INITIAL_SEARCH_HEIGHT:
    #         self.get_logger().info(
    #             f"Search time exceeded {self.ALTITUDE_CHECK_INTERVAL}s. Lowering altitude by {self.ALTITUDE_LOWER_STEP}cm.")
    #         self.action_manager.execute_action(f'down {self.ALTITUDE_LOWER_STEP}')
    #         # Reset the timer for the next check
    #         self.time_search_started = self.get_clock().now()

def main(args=None):
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