#!/usr/bin/env python3
"""Parameter loading and management for mission control."""
from rclpy.node import Node


class ParameterLoader:
    """Manages ROS2 parameters for mission control."""
    
    def __init__(self, node: Node):
        """Initialize parameter loader with ROS2 node."""
        self.node = node
        self._declare_all_parameters()
        self._load_all_parameters()
    
    def _declare_all_parameters(self):
        """Declare all ROS2 parameters with default values."""
        # Identity parameters
        self.node.declare_parameter('drone_id', 'tello1')
        self.node.declare_parameter('priority_markers', list(range(9, 15)))
        self.node.declare_parameter('exit_markers', [])
        
        # TAKING_OFF state parameters
        self.node.declare_parameter('min_takeoff_height', 0.3)
        self.node.declare_parameter('initial_search_height', 1.0)
        self.node.declare_parameter('ascending_speed', 0.2)
        
        # STANDBY state parameters
        self.node.declare_parameter('standby_delay', 5.0)

        # WAYPOINT navigation parameters
        self.node.declare_parameter('enable_waypoint_navigation', False)
        self.node.declare_parameter('waypoint_timeout_s', 10.0)
        self.node.declare_parameter('waypoint_sequence', [''])
        self.node.declare_parameter('waypoint_max_approach_dist', 4.0)
        self.node.declare_parameter('waypoint_step_approach_dist', 1.0)
        
        # SEARCHING state parameters
        self.node.declare_parameter('yaw_speed', 0.5)
        self.node.declare_parameter('forward_speed', 0.2)
        self.node.declare_parameter('sideway_speed', 0.12)
        self.node.declare_parameter('corner_tof_threshold', 0.9)
        self.node.declare_parameter('headon_tof_threshold', 0.5)
        
        # CENTERING state parameters
        self.node.declare_parameter('centering_threshold_x', 0.12)
        self.node.declare_parameter('centering_yaw_kp', 0.38)
        self.node.declare_parameter('centering_yaw_speed', 0.58)
        self.node.declare_parameter('marker_timeout_s', 2.5)
        self.node.declare_parameter('max_approach_dist', 3.5)
        self.node.declare_parameter('step_approach_dist', 0.6)
        self.node.declare_parameter('centering_forward_speed', 0.15)
        
        # APPROACHING state parameters
        self.node.declare_parameter('final_approach_offset', 0.35)

        # PRIORITY_SCANNING parameters
        self.node.declare_parameter('scanning_yaw_speed', 0.4)
        
        # PRECISION_LANDING parameters
        self.node.declare_parameter('precision_landing_threshold_x', 0.12)
        self.node.declare_parameter('precision_landing_threshold_y', 0.12)
        self.node.declare_parameter('precision_landing_max_speed', 0.2)
        self.node.declare_parameter('precision_sideway_kp', 0.78)
        self.node.declare_parameter('precision_forward_kp', 0.78)
        self.node.declare_parameter('precision_landing_timeout_s', 14.0)
        self.node.declare_parameter('recovery_height', 1.5)
        self.node.declare_parameter('landing_height_threshold', 0.90)
        self.node.declare_parameter('descending_speed', 0.15)
    
    def _load_all_parameters(self):
        """Load all parameters into instance variables."""
        # Identity
        self.drone_id = self.node.get_parameter('drone_id').get_parameter_value().string_value
        self.priority_markers = set(self.node.get_parameter('priority_markers').get_parameter_value().integer_array_value)
        self.exit_markers = set(self.node.get_parameter('exit_markers').get_parameter_value().integer_array_value)
        
        # Takeoff/Ascending
        self.min_takeoff_height = self.node.get_parameter('min_takeoff_height').get_parameter_value().double_value
        self.initial_search_height = self.node.get_parameter('initial_search_height').get_parameter_value().double_value
        self.ascending_speed = self.node.get_parameter('ascending_speed').get_parameter_value().double_value
        
        # Standby
        self.standby_delay = self.node.get_parameter('standby_delay').get_parameter_value().double_value

        # Waypoint Navigation
        self.enable_waypoint_navigation = self.node.get_parameter('enable_waypoint_navigation').get_parameter_value().bool_value
        self.waypoint_timeout = self.node.get_parameter('waypoint_timeout_s').get_parameter_value().double_value
        self.waypoint_max_approach_dist = self.node.get_parameter('waypoint_max_approach_dist').get_parameter_value().double_value
        self.waypoint_step_approach_dist = self.node.get_parameter('waypoint_step_approach_dist').get_parameter_value().double_value
        
        # Waypoint sequence - raw string array, parsed by WaypointManager
        waypoint_param = self.node.get_parameter('waypoint_sequence')
        self.waypoint_sequence = [s for s in waypoint_param.get_parameter_value().string_array_value if s]
        
        # Searching
        self.yaw_speed = self.node.get_parameter('yaw_speed').get_parameter_value().double_value
        self.forward_speed = self.node.get_parameter('forward_speed').get_parameter_value().double_value
        self.sideway_speed = self.node.get_parameter('sideway_speed').get_parameter_value().double_value
        self.corner_tof_threshold = self.node.get_parameter('corner_tof_threshold').get_parameter_value().double_value
        self.headon_tof_threshold = self.node.get_parameter('headon_tof_threshold').get_parameter_value().double_value
        
        # Centering
        self.centering_threshold_x = self.node.get_parameter('centering_threshold_x').get_parameter_value().double_value
        self.centering_yaw_kp = self.node.get_parameter('centering_yaw_kp').get_parameter_value().double_value
        self.centering_yaw_speed = self.node.get_parameter('centering_yaw_speed').get_parameter_value().double_value
        self.marker_timeout = self.node.get_parameter('marker_timeout_s').get_parameter_value().double_value
        self.max_approach_dist = self.node.get_parameter('max_approach_dist').get_parameter_value().double_value
        self.step_approach_dist = self.node.get_parameter('step_approach_dist').get_parameter_value().double_value
        self.centering_forward_speed = self.node.get_parameter('centering_forward_speed').get_parameter_value().double_value
        
        # Approaching
        self.final_approach_offset = self.node.get_parameter('final_approach_offset').get_parameter_value().double_value

        # Priority Scanning
        self.scanning_yaw_speed = self.node.get_parameter('scanning_yaw_speed').get_parameter_value().double_value
        
        # Precision Landing
        self.precision_landing_threshold_x = self.node.get_parameter('precision_landing_threshold_x').get_parameter_value().double_value
        self.precision_landing_threshold_y = self.node.get_parameter('precision_landing_threshold_y').get_parameter_value().double_value
        self.precision_landing_max_speed = self.node.get_parameter('precision_landing_max_speed').get_parameter_value().double_value
        self.precision_sideway_kp = self.node.get_parameter('precision_sideway_kp').get_parameter_value().double_value
        self.precision_forward_kp = self.node.get_parameter('precision_forward_kp').get_parameter_value().double_value
        self.precision_landing_timeout = self.node.get_parameter('precision_landing_timeout_s').get_parameter_value().double_value
        self.recovery_height = self.node.get_parameter('recovery_height').get_parameter_value().double_value
        self.landing_height_threshold = self.node.get_parameter('landing_height_threshold').get_parameter_value().double_value
        self.descending_speed = self.node.get_parameter('descending_speed').get_parameter_value().double_value
