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
from tello_msgs.msg import FlightData
from aruco_opencv_msgs.msg import ArucoDetection
from tello_msgs.srv import TelloAction

class MissionState(Enum):
    """Defines the operational states of the drone."""
    SEARCHING = 0
    LOCKING_ON = 1
    CENTERING = 2
    APPROACHING = 3
    LANDING = 4
    EXECUTING_ACTION = 5

class MissionControl(Node):
    """
    The central 'brain' node for the Tello drone's autonomous mission.
    This implementation covers the SEARCHING state logic.
    """
    def __init__(self):
        super().__init__('mission_control')

        # === Parameters ===
        # SEARCHING state parameters
        self.declare_parameter('yaw_speed', 50.0)
        self.declare_parameter('move_speed', 0.0)
        self.declare_parameter('corner_tof_threshold_mm', 900.0)
        self.declare_parameter('headon_tof_threshold_mm', 500.0)
        self.declare_parameter('altitude_check_interval_s', 120.0)
        self.declare_parameter('altitude_lower_step_cm', 20)
        self.declare_parameter('initial_search_height_cm', 60.0)
        # APPROACHING state parameters
        self.declare_parameter('centering_threshold_y', 0.03) # 3cm horizontal tolerance
        self.declare_parameter('centering_yaw_kp', 2.0) # Proportional gain for yaw control
        self.declare_parameter('final_approach_dist_cm', 500.0)
        self.declare_parameter('step_approach_dist_cm', 100.0)
        self.declare_parameter('final_approach_offset_cm', 20.0)
        
        # Assign parameters to member variables
        self.YAW_SPEED = self.get_parameter('yaw_speed').value
        self.MOVE_SPEED = self.get_parameter('move_speed').value
        self.CORNER_TOF_THRESHOLD = self.get_parameter('corner_tof_threshold_mm').value
        self.HEADON_TOF_THRESHOLD = self.get_parameter('headon_tof_threshold_mm').value
        self.ALTITUDE_CHECK_INTERVAL = self.get_parameter('altitude_check_interval_s').value
        self.ALTITUDE_LOWER_STEP = self.get_parameter('altitude_lower_step_cm').value
        self.INITIAL_SEARCH_HEIGHT = self.get_parameter('initial_search_height_cm').value
        self.CENTERING_THRESHOLD_Y = self.get_parameter('centering_threshold_y').value
        self.CENTERING_YAW_KP = self.get_parameter('centering_yaw_kp').value
        self.FINAL_APPROACH_DIST = self.get_parameter('final_approach_dist_cm').value
        self.STEP_APPROACH_DIST = self.get_parameter('step_approach_dist_cm').value
        self.FINAL_APPROACH_OFFSET = self.get_parameter('final_approach_offset_cm').value
        
        # === State & Sensor Data Variables ===
        self.mission_state = MissionState.SEARCHING
        self.latest_tof_mm = None
        self.latest_depth_analysis = None
        self.latest_height_cm = 0.0
        self.time_search_started = self.get_clock().now()
        self.locked_on_marker_id = -1
        self.locked_on_marker_pose = None

        # === ROS2 Communications ===
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/tello1/cmd_vel', 1)
        # self.tof_sub = self.create_subscription(Range, '/tello1/tof', self.tof_callback, 10)
        self.depth_sub = self.create_subscription(DepthMapAnalysis, '/tello1/depth/analysis', self.depth_analysis_callback, qos_profile)
        self.flight_data_sub = self.create_subscription(FlightData, '/tello1/flight_data', self.flight_data_callback, 10)
        # self.aruco_sub = self.create_subscription(ArucoDetection, '/aruco_detections', self.aruco_callback, 10)

        # Service Clients
        self.action_client = self.create_client(TelloAction, '/tello1/tello_action')
        while not self.action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Action service not available, waiting...')

        # self.marker_lock_client = self.create_client(MarkerLock, '/tello1/request_marker_lock')
        # while not self.marker_lock_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Marker Lock service not available, waiting...')

        # === Main Logic Timer ===
        self.timer = self.create_timer(0.25, self.main_logic_loop) # 4 Hz loop
        self.get_logger().info("Mission Control Node has started! Current state: SEARCHING")

    # --- Subscriber Callbacks ---
    def tof_callback(self, msg : Range):
        self.latest_tof_mm = msg.range * 1000.0  # Convert meters to millimeters

    def depth_analysis_callback(self, msg : DepthMapAnalysis):
        self.latest_depth_analysis = msg

    def flight_data_callback(self, msg : FlightData):
        self.latest_height_cm = float(msg.tof)  # height measured by ToF in cm

    # def aruco_callback(self, msg : ArucoDetection):
    #     # Only try to lock on if we are currently searching
    #     if self.mission_state == MissionState.SEARCHING and len(msg.markers) > 0:
    #         self.mission_state = MissionState.LOCKING_ON
    #         try:
    #             first_marker = msg.markers[0]
    #             detected_id = first_marker.id
    #             self.locked_on_marker_pose = first_marker.pose # Store the pose
    #             self.get_logger().info(f"Marker {detected_id} detected. Attempting to lock on...")
    #             self.request_marker_lock(detected_id)
    #         except Exception as e:
    #             self.get_logger().error(f"Error processing ArUco detection message: {e}")
    #             self.mission_state = MissionState.SEARCHING # Go back to searching

    # --- Service Call Logic ---
    def request_marker_lock(self, marker_id):
        req = MarkerLock.Request()
        req.marker_id = marker_id
        future = self.marker_lock_client.call_async(req)
        future.add_done_callback(self.marker_lock_response_callback)

    def marker_lock_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.locked_on_marker_id = future.request.marker_id
                self.get_logger().info(f"Successfully locked on to marker {self.locked_on_marker_id}. Transitioning to CENTERING.")
                self.mission_state = MissionState.CENTERING
            else:
                self.get_logger().info(f"Marker {future.request.marker_id} is not available. Resuming SEARCHING.")
                self.mission_state = MissionState.SEARCHING
        except Exception as e:
            self.get_logger().error(f'Marker lock service call failed: {e}')
            self.mission_state = MissionState.SEARCHING

    def execute_tello_action(self, command: str, next_state_on_success: MissionState):
        if self.mission_state == MissionState.EXECUTING_ACTION: 
            return
        
        self.mission_state = MissionState.EXECUTING_ACTION
        self.get_logger().info(f'Executing discrete action: "{command}"')
        
        req = TelloAction.Request()
        req.cmd = command
        
        future = self.action_client.call_async(req)
        future.add_done_callback(lambda f: self.action_done_callback(f, next_state_on_success))

    def action_done_callback(self, future, next_state):
        # Callback to transition state after a discrete move is complete
        try:
            response = future.result()
            if response.rc == TelloAction.Response.OK:
                self.get_logger().info(f"Action successful. Transitioning to {next_state.name}.")
                self.mission_state = next_state
            else:
                self.get_logger().error("Action failed. Returning to SEARCHING.")
                self.mission_state = MissionState.SEARCHING
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.mission_state = MissionState.SEARCHING

    # --- Main Control Loop & State Logic ---
    def main_logic_loop(self):
        # State machine dispatcher
        if self.mission_state == MissionState.SEARCHING:
            self.run_searching_logic()
        elif self.mission_state == MissionState.CENTERING:
        #     self.run_centering_logic()
        # elif self.mission_state == MissionState.APPROACHING:
        #     self.run_approaching_logic()
        # elif self.mission_state == MissionState.LANDING:
            self.execute_tello_action("land", MissionState.LANDING) # Stay in landing state
        
        # Hover if waiting for a service call to complete (locking on or executing action)
        if self.mission_state in [MissionState.LOCKING_ON, MissionState.EXECUTING_ACTION]:
            self.cmd_vel_pub.publish(Twist())

    def run_searching_logic(self):
        # Wait until all necessary sensor data is available
        # if self.latest_tof_mm is None or self.latest_depth_analysis is None:
        if self.latest_depth_analysis is None:
            self.get_logger().info('SEARCHING: Waiting for sensor data...', throttle_duration_sec=5)
            return

        # Check for altitude adjustment
        # self.check_and_lower_altitude()

        twist_msg = Twist()
        depth = self.latest_depth_analysis

        # 1. ToF Error Check
        # if self.latest_tof_mm > 8000.0:
        #     self.get_logger().warning("ToF out of range or error. Hovering.")
        #     self.cmd_vel_pub.publish(twist_msg)
        #     return
        
        # 2. Obstacle Ahead (Depth Map)
        if depth.middle_center.red > depth.middle_center.blue:
            if depth.middle_left.blue > depth.middle_right.blue:
                self.get_logger().warning("Obstacle detected. Turning Left.")
                twist_msg.angular.z = -self.YAW_SPEED
            else:
                self.get_logger().warning("Obstacle detected. Turning Right.")
                twist_msg.angular.z = self.YAW_SPEED
            self.cmd_vel_pub.publish(twist_msg)
        
        # # 3. Corner Avoidance (Depth Clear + ToF Close)
        # elif depth.middle_center.blue > depth.middle_center.red and self.latest_tof_mm <= self.CORNER_TOF_THRESHOLD:
        #     self.get_logger().info("Corner detected. Executing 150-degree clockwise rotation.")
        #     self.execute_tello_action('cw 150', MissionState.SEARCHING)

        # # 4. Head-on Avoidance (ToF Very Close)
        # elif self.latest_tof_mm <= self.HEADON_TOF_THRESHOLD:
        #     self.get_logger().info("Head-on obstacle detected. Executing 180-degree rotation.")
        #     self.execute_tello_action('cw 180', MissionState.SEARCHING)
        
        # 5. Path Clear
        else:
            self.get_logger().info("Path clear. Moving forward.", throttle_duration_sec=2)
            twist_msg.linear.x = self.MOVE_SPEED
            self.cmd_vel_pub.publish(twist_msg)

    # def run_centering_logic(self):
    #     if self.locked_on_marker_pose is None:
    #         self.get_logger().warn("CENTERING: Lost marker data. Returning to SEARCHING.")
    #         self.mission_state = MissionState.SEARCHING
    #         return

    #     twist_msg = Twist()
    #     y_error = self.locked_on_marker_pose.position.y
        
    #     if abs(y_error) > self.CENTERING_THRESHOLD_Y:
    #         yaw_speed = -np.clip(y_error * self.CENTERING_YAW_KP, -self.YAW_SPEED, self.YAW_SPEED)
    #         twist_msg.angular.z = yaw_speed
    #         self.get_logger().info(f"CENTERING: y_error: {y_error:.2f}m, yaw_speed: {yaw_speed:.2f}", throttle_duration_sec=1.0)
    #     else:
    #         self.get_logger().info("CENTERING: Centered on marker. Transitioning to APPROACHING.")
    #         self.mission_state = MissionState.APPROACHING
        
    #     self.cmd_vel_pub.publish(twist_msg)

    # def run_approaching_logic(self):
    #     if self.locked_on_marker_pose is None:
    #         self.get_logger().warn("APPROACHING: Lost marker data. Returning to SEARCHING.")
    #         self.mission_state = MissionState.SEARCHING
    #         return

    #     dist_3d_cm = self.locked_on_marker_pose.position.z * 100
    #     if dist_3d_cm < self.latest_height_cm:
    #          self.get_logger().warn("APPROACHING: 3D distance is less than height, cannot compute 2D distance. Returning to SEARCHING")
    #          self.mission_state = MissionState.SEARCHING
    #          return

    #     dist_2d_cm = math.sqrt(dist_3d_cm**2 - self.latest_height_cm**2)
    #     self.get_logger().info(f"APPROACHING: 2D distance to marker: {dist_2d_cm:.1f} cm")

    #     if dist_2d_cm >= self.FINAL_APPROACH_DIST:
    #         self.get_logger().info(f"Step approach: moving forward {self.STEP_APPROACH_DIST} cm.")
    #         self.execute_tello_action(f'forward {self.STEP_APPROACH_DIST}', MissionState.CENTERING)
    #     else:
    #         move_dist = int(dist_2d_cm - self.FINAL_APPROACH_OFFSET)
    #         if move_dist > 20:
    #             self.get_logger().info(f"Final approach: moving forward {move_dist} cm.")
    #             self.execute_tello_action(f'forward {move_dist}', MissionState.LANDING)
    #         else:
    #             self.get_logger().info("Approach complete. Transitioning to LANDING.")
    #             self.mission_state = MissionState.LANDING

    # def check_and_lower_altitude(self):
    #     """Checks if enough time has passed to lower the drone's search altitude."""
    #     elapsed_time = (self.get_clock().now() - self.time_search_started).nanoseconds / 1e9
        
    #     if elapsed_time > self.ALTITUDE_CHECK_INTERVAL and self.latest_height_cm >= self.INITIAL_SEARCH_HEIGHT:
    #         self.get_logger().info(
    #             f"Search time exceeded {self.ALTITUDE_CHECK_INTERVAL}s. Lowering altitude by {self.ALTITUDE_LOWER_STEP}cm.")
    #         self.execute_tello_action(f'down {self.ALTITUDE_LOWER_STEP}')
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