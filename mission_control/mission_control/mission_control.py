#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum
import time

# Import ROS2 message and service types
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from midas_msgs.msg import DepthMapAnalysis
from tello_msgs.msg import FlightData
from tello_msgs.srv import TelloAction

class MissionState(Enum):
    """Defines the operational states of the drone."""
    SEARCHING = 0
    EXECUTING_ACTION = 1
    # Other states like APPROACHING, CENTERING will be added later

class MissionControl(Node):
    """
    The central 'brain' node for the Tello drone's autonomous mission.
    This implementation covers the SEARCHING state logic.
    """
    def __init__(self):
        super().__init__('mission_control')

        # === Parameters ===
        self.declare_parameters(
            namespace='',
            parameters=[
                ('yaw_speed', 50),
                ('move_speed', 20),
                ('corner_tof_threshold_mm', 900.0),
                ('headon_tof_threshold_mm', 500.0),
                ('altitude_check_interval_s', 120.0),
                ('altitude_lower_step_cm', 20),
                ('initial_search_height_cm', 60.0)
            ])
        
        # Assign parameters to member variables for easy access
        self.YAW_SPEED = self.get_parameter('yaw_speed').value
        self.MOVE_SPEED = self.get_parameter('move_speed').value
        self.CORNER_TOF_THRESHOLD = self.get_parameter('corner_tof_threshold_mm').value
        self.HEADON_TOF_THRESHOLD = self.get_parameter('headon_tof_threshold_mm').value
        self.ALTITUDE_CHECK_INTERVAL = self.get_parameter('altitude_check_interval_s').value
        self.ALTITUDE_LOWER_STEP = self.get_parameter('altitude_lower_step_cm').value
        self.INITIAL_SEARCH_HEIGHT = self.get_parameter('initial_search_height_cm').value
        
        # === State & Sensor Data Variables ===
        self.mission_state = MissionState.SEARCHING
        self.latest_tof_mm = None
        self.latest_depth_analysis = None
        self.latest_height_cm = 0.0
        self.time_search_started = self.get_clock().now()

        # === ROS2 Communications ===
        self.cmd_vel_pub = self.create_publisher(Twist, '/tello1/cmd_vel', 10)
        
        # self.tof_sub = self.create_subscription(
        #     Range, '/tello1/tof', self.tof_callback, 10)
        self.depth_sub = self.create_subscription(
            DepthMapAnalysis, '/tello1/depth/analysis', self.depth_analysis_callback, 10)
        # self.flight_data_sub = self.create_subscription(
        #     FlightData, '/tello1/flight_data', self.flight_data_callback, 10)

        self.action_client = self.create_client(TelloAction, '/tello1/tello_action')
        while not self.action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Action service not available, waiting...')

        # === Main Logic Timer ===
        self.timer = self.create_timer(0.25, self.main_logic_loop) # 4 Hz loop
        self.get_logger().info("Mission Control Node has started!")

    # --- Subscriber Callbacks ---
    # def tof_callback(self, msg):
    #     self.latest_tof_mm = msg.range * 1000.0  # Convert meters to millimeters

    def depth_analysis_callback(self, msg : DepthMapAnalysis):
        self.latest_depth_analysis = msg

    # def flight_data_callback(self, msg):
    #     self.latest_height_cm = float(msg.h)

    # --- Service Call Logic ---
    def execute_tello_action(self, command: str):
        """Asynchronously calls a Tello action and manages state."""
        if self.mission_state == MissionState.EXECUTING_ACTION:
            self.get_logger().warn(f"Already executing an action, ignoring command: '{command}'")
            return

        self.mission_state = MissionState.EXECUTING_ACTION
        self.get_logger().info(f'Executing discrete action: "{command}"')
        
        req = TelloAction.Request()
        req.cmd = command
        
        future = self.action_client.call_async(req)
        future.add_done_callback(self.action_done_callback)

    def action_done_callback(self, future):
        """Resets state to SEARCHING after a discrete action is completed."""
        try:
            response = future.result()
            if response.rc == TelloAction.Response.OK:
                self.get_logger().info("Action executed successfully.")
            else:
                self.get_logger().error("Action failed to execute.")
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        
        time.sleep(1.0) # Small delay to stabilize before resuming search
        self.mission_state = MissionState.SEARCHING
        self.get_logger().info('Resuming SEARCHING state.')

    # --- Main Loop ---
    def main_logic_loop(self):
        """This function is called periodically and contains the main state machine."""
        # Don't do anything if we are waiting for a discrete action to complete
        if self.mission_state != MissionState.SEARCHING:
            self.cmd_vel_pub.publish(Twist()) # Send hover command while waiting
            return

        # Wait until all necessary sensor data is available
        # if self.latest_tof_mm is None or self.latest_depth_analysis is None:
        if self.latest_depth_analysis is None:
            self.get_logger().info('Waiting for initial sensor data...', throttle_duration_sec=5)
            return

        # Check for altitude adjustment
        # self.check_and_lower_altitude()

        # === Ported Navigation Logic from nav_with_depthmap_tof ===
        twist_msg = Twist()
        depth = self.latest_depth_analysis

        # 1. ToF Error Check
        # if self.latest_tof_mm > 8000.0:
        #     self.get_logger().warn("ToF out of range or error. Hovering.")
        #     self.cmd_vel_pub.publish(twist_msg)
        #     return
        
        # 2. Obstacle Ahead (Depth Map)
        if depth.middle_center.red > depth.middle_center.blue:
            if depth.middle_left.blue > depth.middle_right.blue:
                self.get_logger().info("Obstacle detected. Turning Left.")
                twist_msg.angular.z = -self.YAW_SPEED
            else:
                self.get_logger().info("Obstacle detected. Turning Right.")
                twist_msg.angular.z = self.YAW_SPEED
            self.cmd_vel_pub.publish(twist_msg)
        
        # # 3. Corner Avoidance (Depth Clear + ToF Close)
        # elif depth.middle_center.blue > depth.middle_center.red and self.latest_tof_mm <= self.CORNER_TOF_THRESHOLD:
        #     self.get_logger().info("Corner detected. Executing 150-degree clockwise rotation.")
        #     self.execute_tello_action('cw 150')

        # # 4. Head-on Avoidance (ToF Very Close)
        # elif self.latest_tof_mm <= self.HEADON_TOF_THRESHOLD:
        #     self.get_logger().info("Head-on obstacle detected. Executing 180-degree rotation.")
        #     self.execute_tello_action('rcw 180') # Assuming 'rcw' for rotate_clockwise
        
        # 5. Path Clear
        else:
            self.get_logger().info("Path clear. Moving forward.", throttle_duration_sec=2)
            twist_msg.linear.x = self.MOVE_SPEED
            self.cmd_vel_pub.publish(twist_msg)

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