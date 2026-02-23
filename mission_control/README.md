# mission_control

ROS 2 Python package implementing a **state-machine-based autonomous mission controller** for DJI Tello EDU drones. Handles the full mission lifecycle — takeoff, waypoint navigation, ArUco marker search, swarm-coordinated target acquisition, precision landing, and mission reset — with depth-based obstacle avoidance and multi-drone coordination.

---

## Table of Contents

- [Architecture](#architecture)
- [Package Structure](#package-structure)
- [State Machine](#state-machine)
- [Modules](#modules)
  - [Orchestrator Node](#orchestrator-node)
  - [State Machine](#state-machine-module)
  - [Controller](#controller)
  - [ArUco](#aruco)
  - [Utils](#utils)
- [Topics, Services & Interfaces](#topics-services--interfaces)
- [Parameters](#parameters)
- [Usage](#usage)
- [Configuration](#configuration)
- [Adding a New State](#adding-a-new-state)

---

## Architecture

The package follows a **thin-orchestrator + State Pattern** design:

```
mission_control_node.py          (thin orchestrator)
├── MissionManager               (state registry & delegation)
│   ├── MissionContext           (shared runtime state, 4 fields)
│   └── BaseState subclasses     (15 state classes)
├── DroneInterface               (control + telemetry facade)
│   ├── ActionManager            (async SDK command lifecycle)
│   └── YawController            (heading-feedback rotation)
├── ArucoMarkerHandler           (marker selection + swarm reservation)
├── ExitMarkerHandler            (exit boundary avoidance)
├── WaypointManager              (sequential ArUco waypoint tracking)
└── ParameterLoader              (ROS 2 parameter declarations)
```

**Key design principles:**

1. **States are stateless** — no instance variables; shared data lives in `MissionContext` (dataclass with `__slots__`).
2. **Return-based transitions** — `execute() -> Optional[MissionState]`; return the next state or `None` to stay.
3. **Separation of concerns** — the node handles ROS 2 I/O (subscriptions, services, callbacks); state classes contain only control logic.
4. **Three transition mechanisms** — synchronous (`execute()` return), asynchronous (action/yaw callbacks), and event-driven (ArUco detection callback).

---

## Package Structure

```
mission_control/
├── mission_control_node.py              # Thin orchestrator node
├── state_machine/
│   ├── states.py                        # MissionState enum (17 states)
│   ├── base_state.py                    # Abstract BaseState with 6 slots
│   ├── mission_context.py               # Shared runtime dataclass
│   ├── mission_manager.py               # State registry + execute_state()
│   └── logic/
│       ├── taking_off.py                # S1  — Takeoff command
│       ├── ascending.py                 # S2  — Climb to search height
│       ├── standby.py                   # S3  — Configurable pre-mission delay
│       ├── waypoint_centering.py        # S4  — Yaw align with waypoint marker
│       ├── waypoint_approaching.py      # S5  — Forward movement to waypoint
│       ├── waypoint_action.py           # S6  — Post-waypoint SDK action
│       ├── searching.py                 # S7  — Autonomous exploration
│       ├── centering.py                 # S9  — Yaw align on landing marker
│       ├── approaching.py              # S10 — Forward to landing marker
│       ├── priority_scanning.py        # S16 — 360° scan for priority markers
│       ├── camera_switching.py         # S11 — Switch to downward camera
│       ├── precision_landing.py        # S12 — PID alignment over marker
│       ├── landing.py                  # S13 — SDK land command
│       ├── completing_mission.py       # S14 — Mark landing in swarm server
│       └── resetting.py               # S15 — Reset context, return to IDLE
├── controller/
│   ├── drone_interface.py               # High-level control + sensor facade
│   ├── action_manager.py               # Async SDK command lifecycle + retry
│   └── yaw_controller.py               # Heading-feedback angle rotation
├── aruco/
│   ├── aruco_marker_handler.py          # Marker selection + swarm coordination
│   ├── exit_marker_handler.py           # Exit boundary proximity detection
│   └── waypoint_manager.py             # Sequential waypoint tracking
└── utils/
    └── parameter_loader.py              # ROS 2 parameter management
```

---

## State Machine

17 states organized into 6 phases:

| Phase | States | Description |
|-------|--------|-------------|
| **Startup** | IDLE → TAKING_OFF → ASCENDING → STANDBY | Takeoff and climb to search altitude |
| **Waypoint Nav** | WAYPOINT_CENTERING → WAYPOINT_APPROACHING → WAYPOINT_ACTION | Sequential ArUco-guided waypoint navigation |
| **Target Acquisition** | SEARCHING → LOCKING_ON → CENTERING → APPROACHING | Autonomous exploration with swarm-coordinated marker locking |
| **Pre-Landing** | PRIORITY_SCANNING → CAMERA_SWITCHING | 360° scan for higher-priority markers before committing |
| **Precision Landing** | PRECISION_LANDING → LANDING | PID alignment on downward camera + land |
| **Completion** | COMPLETING_MISSION → RESETTING → IDLE | Mark landing status, reset, return to idle |

> **LOCKING_ON (S8)** is a special node-level state with no `BaseState` subclass — the drone hovers while the swarm marker server processes the reservation asynchronously.

For the complete state diagram, transition table, and detailed descriptions, see [docs/state_machine_diagram.md](docs/state_machine_diagram.md).

---

## Modules

### Orchestrator Node

**`mission_control_node.py`** — `MissionControl(Node)`

The thin orchestrator that wires everything together. It does **not** contain control logic; instead it:

- Runs the **main loop at 4 Hz** — checks timeouts, guards on `drone.is_busy()`, delegates to `MissionManager.execute_state()`.
- Handles **ArUco callbacks** — routes detections to different handlers based on current state (waypoint pose updates, search detection → LOCKING_ON, priority marker switching, locked marker pose tracking).
- Exposes **services** — `/takeoff` (Trigger) and `/land` (Trigger) for external mission control.
- **Renews marker reservations** every 2 seconds via heartbeat during approach/landing states.
- Manages **node-level transitions** that can't be expressed as `execute()` returns (async callbacks, ArUco events).

### State Machine Module

| File | Class | Role |
|------|-------|------|
| `states.py` | `MissionState(Enum)` | 17-variant enum (IDLE=0 through PRIORITY_SCANNING=16) |
| `base_state.py` | `BaseState(ABC)` | Abstract base with 6 `__slots__`: `node`, `drone`, `marker_handler`, `waypoint_manager`, `params`, `context` |
| `mission_context.py` | `MissionContext` | `@dataclass(slots=True)` with 4 fields: `standby_start_time`, `is_near_exit`, `is_blind_landing`, `precision_landing_initialized` |
| `mission_manager.py` | `MissionManager` | Creates all 15 state instances, exposes `execute_state()` and context properties |

### Controller

| File | Class | Role |
|------|-------|------|
| `drone_interface.py` | `DroneInterface` | Facade over publishers, subscribers, ActionManager, and YawController. Subscribes to `flight_data`, `ext_tof`, `depth/analysis`. Publishes to `cmd_vel`. Provides `hover()`, `move_*()`, `execute_action()`, `yaw_*_by_angle()`. |
| `action_manager.py` | `ActionManager` | Manages the full lifecycle of a Tello SDK command: service request → acceptance → response monitoring → timeout/retry. Configurable per-command timeout and max retries. |
| `yaw_controller.py` | `YawController` | Angle-based rotation using continuous `cmd_vel` yaw velocity with heading feedback. Adaptive slowdown near target. Replaces unreliable SDK `cw`/`ccw` commands. |

### ArUco

| File | Class | Role |
|------|-------|------|
| `aruco_marker_handler.py` | `ArucoMarkerHandler` | Core marker coordination: selection (priority-first, then closest), swarm reservation via `/reserve_marker` service, heartbeat renewal, pose tracking, timeout detection. Thread-safe unavailable marker tracking. |
| `exit_marker_handler.py` | `ExitMarkerHandler` | Detects proximity to exit/boundary markers (IDs ≥ cutoff) within 5m. Sets `is_near_exit` flag to trigger avoidance turn. |
| `waypoint_manager.py` | `WaypointManager` | Parses waypoint sequences (`"id:50,action:ccw 90"`), tracks current index, updates marker pose from ArUco detections, provides `advance_to_next()` and timeout checking. |

### Utils

| File | Class | Role |
|------|-------|------|
| `parameter_loader.py` | `ParameterLoader` | Declares and loads all ROS 2 parameters with defaults. Single source of truth for configuration. |

---

## Topics, Services & Interfaces

### Subscribed Topics

| Topic | Type | QoS | Purpose |
|-------|------|-----|---------|
| `aruco_detections` | `aruco_opencv_msgs/ArucoDetection` | Best-effort | ArUco marker poses from vision pipeline |
| `flight_data` | `tello_msgs/FlightData` | Best-effort | Height (ToF), heading, battery from drone |
| `ext_tof` | `sensor_msgs/Range` | Best-effort | Forward-facing external ToF sensor |
| `depth/analysis` | `midas_msgs/DepthMapAnalysis` | Best-effort | MiDaS depth map 9-region analysis |
| `/unavailable_markers` | `std_msgs/Int32MultiArray` | Default | Reserved/landed markers from swarm server |

### Published Topics

| Topic | Type | QoS | Purpose |
|-------|------|-----|---------|
| `cmd_vel` | `geometry_msgs/Twist` | Default | Velocity commands to drone driver |
| `mission_state` | `std_msgs/UInt8` | Transient local | Current state for monitoring dashboards |
| `locked_marker_id` | `std_msgs/Int32` | Transient local | Currently locked ArUco marker ID |
| `/marker_heartbeat` | `swarm_interfaces/MarkerHeartbeat` | Reliable | Reservation renewal (every 2s) |

### Service Servers

| Service | Type | Purpose |
|---------|------|---------|
| `takeoff` | `std_srvs/Trigger` | Initiate mission (IDLE → TAKING_OFF) |
| `land` | `std_srvs/Trigger` | Emergency landing (any active → LANDING) |

### Service Clients

| Service | Type | Purpose |
|---------|------|---------|
| `tello_action` | `tello_msgs/TelloAction` | Send SDK commands to drone driver |
| `/reserve_marker` | `swarm_interfaces/ReserveMarker` | Reserve ArUco marker in swarm server |
| `/unreserve_marker` | `swarm_interfaces/ReserveMarker` | Release marker reservation |
| `/mark_landed` | `swarm_interfaces/MarkLanded` | Report successful landing on marker |

### Subscribed Response Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `tello_response` | `tello_msgs/TelloResponse` | SDK command completion acknowledgement |

> All topics except those prefixed with `/` are relative to the drone namespace (e.g., `/tello1/cmd_vel`).

---

## Parameters

All parameters are declared in `ParameterLoader` with sensible defaults and are typically overridden from YAML config files.

### Identity

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `drone_id` | string | `"tello1"` | Unique drone identifier for swarm coordination |
| `priority_markers` | int[] | `[9,10,11,12,13,14]` | High-priority ArUco marker IDs (fire markers) |
| `exit_markers` | int[] | `[]` | Boundary marker IDs to avoid during search |

### Startup & Navigation

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_takeoff_height` | double | `0.3` | Minimum height (m) to consider drone airborne |
| `initial_search_height` | double | `1.0` | Target altitude (m) before starting search |
| `ascending_speed` | double | `0.2` | Vertical climb speed (m/s) |
| `standby_delay` | double | `5.0` | Pre-mission delay (s) for staggered swarm takeoff |

### Waypoint Navigation

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_waypoint_navigation` | bool | `false` | Enable sequential waypoint phase |
| `waypoint_sequence` | string[] | `[""]` | Waypoint definitions (see [Configuration](#configuration)) |
| `waypoint_timeout_s` | double | `10.0` | Marker search timeout per waypoint (s) |
| `waypoint_max_approach_dist` | double | `4.0` | Maximum forward distance to waypoint (m) |
| `waypoint_step_approach_dist` | double | `1.0` | Step distance per approach iteration (m) |

### Search & Obstacle Avoidance

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `yaw_speed` | double | `0.5` | Yaw rotation speed (rad/s) |
| `forward_speed` | double | `0.2` | Forward exploration speed (m/s) |
| `sideway_speed` | double | `0.12` | Lateral movement speed (m/s) |
| `corner_tof_threshold` | double | `0.9` | ToF distance (m) triggering 137° corner avoidance |
| `headon_tof_threshold` | double | `0.5` | ToF distance (m) triggering 180° turn |

### Centering & Approach

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `centering_threshold_x` | double | `0.12` | Horizontal alignment tolerance (m) |
| `centering_yaw_kp` | double | `0.38` | Proportional gain for yaw centering |
| `centering_yaw_speed` | double | `0.58` | Max yaw speed during centering (rad/s) |
| `centering_forward_speed` | double | `0.15` | Forward creep speed during centering (m/s) |
| `marker_timeout_s` | double | `2.5` | Marker loss timeout before fallback (s) |
| `max_approach_dist` | double | `3.5` | Maximum approach distance cap (m) |
| `step_approach_dist` | double | `0.6` | Incremental approach step distance (m) |
| `final_approach_offset` | double | `0.35` | Offset subtracted from forward distance to stop above marker (m) |
| `scanning_yaw_speed` | double | `0.4` | Yaw speed during 360° priority scan (rad/s) |

### Precision Landing

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `precision_landing_threshold_x` | double | `0.12` | X-axis alignment tolerance (m) |
| `precision_landing_threshold_y` | double | `0.12` | Y-axis alignment tolerance (m) |
| `precision_landing_max_speed` | double | `0.2` | Max alignment speed (m/s) |
| `precision_forward_kp` | double | `0.78` | Forward proportional gain |
| `precision_sideway_kp` | double | `0.78` | Lateral proportional gain |
| `precision_landing_timeout_s` | double | `14.0` | Full timeout before blind landing (s) |
| `recovery_height` | double | `1.5` | Target height for recovery ascent (m) |
| `landing_height_threshold` | double | `0.90` | Height below which centered = land (m) |
| `descending_speed` | double | `0.15` | Descent speed during precision landing (m/s) |

---

## Usage

### Prerequisites

The following must be running in the same drone namespace before launching mission control:

- **tello_driver** — Tello SDK bridge (UDP, video, telemetry)
- **ArUco detector** — publishes `aruco_detections`
- **MiDaS depth** — publishes `depth/analysis`
- **Swarm servers** — `/reserve_marker`, `/unreserve_marker`, `/mark_landed` services

### Build

```bash
cd ~/tello_ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mission_control
source install/setup.bash
```

### Launch (Single Drone)

```bash
ros2 launch tello_bringup mission_control_launch.py drone_ns:=tello1
```

### Launch (Multi-Drone Swarm)

```bash
# Terminal 1: Swarm coordination servers
ros2 launch tello_bringup swarm_servers_launch.py

# Terminal 2: All drone drivers + perception
ros2 launch tello_bringup multi_drone_launch.py

# Terminal 3+: Mission control per drone
ros2 launch tello_bringup mission_control_launch.py drone_ns:=tello1
ros2 launch tello_bringup mission_control_launch.py drone_ns:=tello4
ros2 launch tello_bringup mission_control_launch.py drone_ns:=tello7
```

### Trigger Mission

```bash
# Start mission (IDLE → TAKING_OFF)
ros2 service call /tello1/takeoff std_srvs/srv/Trigger

# Emergency landing (any state → LANDING)
ros2 service call /tello1/land std_srvs/srv/Trigger
```

### Monitor

```bash
# Current state (numeric enum value)
ros2 topic echo /tello1/mission_state

# Which marker is locked
ros2 topic echo /tello1/locked_marker_id

# Swarm marker availability
ros2 topic echo /unavailable_markers
```

---

## Configuration

### Waypoint Sequence Format

Waypoints are defined as string arrays in YAML:

```yaml
waypoint_sequence:
  - "id:50,action:ccw 90"     # Navigate to marker 50, then rotate 90° CCW
  - "id:51,action:none"       # Navigate to marker 51, no post-action
  - "id:none,action:forward 400"  # No navigation, just fly forward 400cm
  - "id:none,action:cw 45"    # No navigation, just rotate 45° CW
```

- `id:none` — action-only waypoint (skips marker navigation, executes action immediately).
- `action:none` — no post-arrival action.
- Rotation actions (`cw`, `ccw`) use the `YawController`; linear actions use `ActionManager`.

### Per-Drone Mission Config

Mission parameters are loaded from `tello_bringup/missions/mission_waypoints.yaml`, namespaced per drone:

```yaml
tello7:
  mission_control:
    ros__parameters:
      initial_search_height: 1.10
      enable_waypoint_navigation: true
      standby_delay: 10.0
      waypoint_sequence:
        - "id:5,action:cw 90"
        - "id:1,action:ccw 90"
        - "id:none,action:forward 400"
```

---

## Adding a New State

1. **Create the state class** in `state_machine/logic/`:

```python
# state_machine/logic/my_new_state.py
from typing import Optional
from ..states import MissionState
from ..base_state import BaseState

class MyNewState(BaseState):
    """Description of what this state does."""

    def execute(self) -> Optional[MissionState]:
        # Access shared state via self.context
        # Access drone via self.drone
        # Access parameters via self.params
        # Return MissionState.NEXT_STATE to transition, or None to stay
        return MissionState.SEARCHING
```

2. **Add the enum variant** in `states.py`:

```python
class MissionState(Enum):
    ...
    MY_NEW_STATE = 17
```

3. **Register in `MissionManager._create_states()`** in `mission_manager.py`:

```python
MissionState.MY_NEW_STATE: MyNewState(*common_args),
```

4. **Export** in `state_machine/logic/__init__.py`:

```python
from .my_new_state import MyNewState
```

5. **Update documentation** in `docs/state_machine_diagram.md`.

---

## Dependencies

| Package | Purpose |
|---------|---------|
| `rclpy` | ROS 2 Python client |
| `geometry_msgs` | Twist, Pose |
| `sensor_msgs` | Range (ToF) |
| `std_msgs` | UInt8, Int32, Int32MultiArray |
| `std_srvs` | Trigger service |
| `tello_msgs` | FlightData, TelloAction, TelloResponse |
| `midas_msgs` | DepthMapAnalysis |
| `aruco_opencv_msgs` | ArucoDetection |
| `swarm_interfaces` | ReserveMarker, MarkLanded, MarkerHeartbeat |

---

## License

Apache-2.0
