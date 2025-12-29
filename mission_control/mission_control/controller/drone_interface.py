#!/usr/bin/env python3
"""Drone interface for high-level control and telemetry access."""
from typing import Optional, Callable
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from midas_msgs.msg import DepthMapAnalysis
from tello_msgs.msg import FlightData
from tello_msgs.srv import TelloAction

from .action_manager import ActionManager
from ..state_machine import MissionState


class DroneInterface:
    """Provides high-level interface to drone control and sensor data."""
    
    def __init__(
        self,
        node: Node,
        on_action_success: Callable,
        on_action_fail: Callable,
        action_service_name: str = 'tello_action',
        response_topic_name: str = 'tello_response',
    ):
        """
        Initialize drone interface with complete I/O setup.
        
        Args:
            node: ROS2 node for logging and communication
            on_action_success: Callback for successful actions (next_state)
            on_action_fail: Callback for failed actions (fallback_state)
            action_service_name: Name of TelloAction service (default: 'tello_action')
            response_topic_name: Name of TelloResponse topic (default: 'tello_response')
        """
        self.node = node
        self.action_service_name = action_service_name
        self.response_topic_name = response_topic_name

        self._init_sensor_vars()
        self._setup_pubs_subs()
        self._setup_service_clients()
        
        # Action manager
        self.action_manager = ActionManager(
            node,
            self._action_client,
            response_topic_name,
            on_action_success,
            on_action_fail,
            self._invalidate_sensor_data
        )

    def _init_sensor_vars(self):
        # Sensor data
        self.latest_tof: Optional[float] = None
        self.latest_depth_analysis: Optional[DepthMapAnalysis] = None
        self.latest_height: float = 0.0
        self.latest_heading: int = 0
        self.latest_battery: int = 100

    def _setup_pubs_subs(self):
        # QoS profile for sensor topics (BEST_EFFORT for real-time telemetry)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 1)
        
        # Subscribers for drone telemetry
        self._tof_sub = self.node.create_subscription(
            Range,
            'ext_tof',
            self._tof_callback,
            qos_profile
        )
        self._depth_sub = self.node.create_subscription(
            DepthMapAnalysis,
            'depth/analysis',
            self._depth_analysis_callback,
            qos_profile
        )
        self._flight_data_sub = self.node.create_subscription(
            FlightData,
            'flight_data',
            self._flight_data_callback,
            qos_profile
        )

    def _setup_service_clients(self):
        # Initialize TelloAction service client
        self._action_client = self.node.create_client(TelloAction, self.action_service_name)
        while not self._action_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'Waiting for {self.action_service_name} service...')

    def _invalidate_sensor_data(self):
        """Clear critical sensor data before executing actions."""
        self.latest_tof = None
        self.latest_depth_analysis = None
    
    # ========================================================================
    # ROS2 SENSOR CALLBACKS
    # ========================================================================
    
    def _tof_callback(self, msg: Range):
        """Internal callback for ToF sensor updates."""
        self.latest_tof = msg.range
    
    def _depth_analysis_callback(self, msg: DepthMapAnalysis):
        """Internal callback for depth analysis updates."""
        self.latest_depth_analysis = msg
    
    def _flight_data_callback(self, msg: FlightData):
        """Internal callback for flight data updates with battery monitoring."""
        self.latest_height = float(msg.tof) / 100.0
        self.latest_heading = msg.yaw
        self.latest_battery = msg.bat
        
        # Battery status warnings
        if self.is_battery_critical():
            self.node.get_logger().error(
                f"CRITICAL: Battery at {self.latest_battery}%",
                throttle_duration_sec=10.0
            )
        elif self.is_battery_low():
            self.node.get_logger().warning(
                f"Battery at {self.latest_battery}%",
                throttle_duration_sec=20.0
            )
    
    # ========================================================================
    # COMMAND INTERFACE
    # ========================================================================
    
    def publish_velocity(self, twist: Twist):
        """Publish velocity command to drone."""
        self.cmd_vel_pub.publish(twist)
    
    def hover(self):
        """Command drone to hover in place."""
        self.cmd_vel_pub.publish(Twist())
    
    def move(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, yaw: float = 0.0):
        """
        Convenience method to publish velocity command.
        
        Args:
            x: Forward/backward velocity (m/s)
            y: Left/right velocity (m/s)
            z: Up/down velocity (m/s)
            yaw: Yaw rate (rad/s)
        """
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        twist.angular.z = yaw
        self.cmd_vel_pub.publish(twist)

    def move_forward(self, speed: float = 0.0):
        self.move(x=speed)

    def move_back(self, speed: float = 0.0):
        self.move(x=-speed)

    def move_up(self, speed: float = 0.0):
        self.move(z=speed)

    def move_down(self, speed: float = 0.0):
        self.move(z=-speed)

    def move_right(self, speed: float = 0.0):
        self.move(y=speed)

    def move_left(self, speed: float = 0.0):
        self.move(y=-speed)

    def yaw_right(self, speed: float = 0.0):
        self.move(yaw=speed)

    def yaw_left(self, speed: float = 0.0):
        self.move(yaw=-speed)

    def execute_action(
        self,
        command: str,
        next_state: MissionState,
        fallback_state: MissionState,
        timeout: float = None,
        max_retries: int = None
    ):
        """
        Execute a drone action command with optional timeout and retry configuration.
        
        Args:
            command: Tello SDK command string (e.g., 'takeoff', 'forward 20')
            next_state: State to transition to on success
            fallback_state: State to transition to on failure
            timeout: Command timeout in seconds (default: 10.0)
            max_retries: Maximum retry attempts on failure (default: 0, no retries)
        """
        self.action_manager.execute_action(
            command, next_state, fallback_state, timeout, max_retries
        )
    
    def is_busy(self) -> bool:
        """Check if an action is currently executing."""
        return self.action_manager.is_busy()
    
    def check_timeout(self):
        """Check and handle action timeouts."""
        self.action_manager.check_timeout()
    
    # ========================================================================
    # SENSOR QUERY INTERFACE
    # ========================================================================
    
    def has_sensor_data(self) -> bool:
        """Check if critical sensor data is available."""
        return self.latest_tof is not None and self.latest_depth_analysis is not None
    
    def get_tof_distance(self) -> Optional[float]:
        """Get forward ToF distance in meters (None if unavailable)."""
        return self.latest_tof
    
    def get_height(self) -> float:
        """Get current height above ground in meters."""
        return self.latest_height
    
    def get_heading(self) -> int:
        """Get current heading (yaw) in degrees."""
        return self.latest_heading
    
    def get_battery_percent(self) -> int:
        """Get current battery percentage (0-100)."""
        return self.latest_battery
    
    def is_battery_critical(self, threshold: int = 20) -> bool:
        """Check if battery is below critical threshold."""
        return self.latest_battery < threshold
    
    def is_battery_low(self, threshold: int = 50) -> bool:
        """Check if battery is below low threshold."""
        return self.latest_battery < threshold
