# BRACU DUBURI — AUV ROS2 Control Stack

> **Team**: BRACU Duburi, Bangladesh | [bracuduburi.com/auv/4.2](https://bracuduburi.com/auv/4.2)
> **Competitions**: RoboSub (8th 2025, 2nd 2023), SAUVC
> **This codebase**: `~/Ros_workspaces/duburi_ws` (ROS2 Humble, Ubuntu 22.04 in distrobox)

---

## 1. Hardware Overview

| Component | Spec | Notes |
|---|---|---|
| **Flight Controller (FC)** | Pixhawk 2.4.8 | Running ArduSub firmware (4.x) |
| **Main SBC** | Jetson Orin Nano (Developer Kit) | Runs all ROS2 nodes, mission code |
| **Companion** | Raspberry Pi 4B 8GB | BlueOS: monitoring, calibration, video |
| **Thrusters** | 8× Blue Robotics T200 | vectored_6dof frame (BlueROV2 Heavy) |
| **Cameras** | Blue Robotics Low-Light (forward + downward) | GStreamer pipelines |
| **Depth Sensor** | Bar30 | Via ArduSub AHRS2 altitude |
| **DVL** | Nortek Nucleus1000 | TCP/Serial at 192.168.2.201 |
| **Frame** | BlueROV2 Heavy | 6-DOF: pitch/roll/yaw/fwd/lat/depth |

---

## 2. Network Topology (AUV Internal Ethernet)

```
[Ethernet Switch — heart of AUV comms]
       ├── Jetson Orin Nano  → 192.168.2.69   (static, SSH/VNC from ground)
       ├── Raspberry Pi 4B   → 192.168.2.1    (BlueOS companion)
       │      Gateway         → 192.168.2.2
       ├── DVL Nucleus1000   → 192.168.2.201
       └── Pixhawk 2.4.8     → via BlueOS MAVLink endpoint

MAVLink Endpoint (from BlueOS):
  Name: "inspector"  |  Type: UDP Client
  IP: 192.168.2.69 (Jetson)  |  Port: 14550

Ground Station → Remote Desktop to Jetson (192.168.2.69)
              → BlueOS UI via Raspberry Pi (192.168.2.1)
```

**Connection string in code (POOL mode):**
```python
CONNECTION_STRING = "udpin:0.0.0.0:14550"  # Jetson listens, BlueOS sends
```

---

## 3. Operating Modes

The codebase supports three modes — default is **SIM** so development works immediately.

| Mode | Connection String | When Used |
|---|---|---|
| **SIM** *(default)* | `udpin:0.0.0.0:14550` | Docker dev + Gazebo SITL |
| **POOL** | `udpin:0.0.0.0:14550` | Pool testing (tether connected) |
| **HARDWARE** | `udpin:0.0.0.0:14550` | Autonomous mission (tether removed) |

SIM startup commands (run before ROS2 nodes):
```bash
# Terminal 1: ArduSub SITL
sim_vehicle.py -L RATBeach -v ArduSub -f vectored_6dof --model=JSON \
  --out=udp:0.0.0.0:14550 --out=udp:127.0.0.1:14551 --console

# Terminal 2: Gazebo
cd ~/Ros_workspaces/colcon_ws
gz sim -v 3 -r src/bluerov2_gz/worlds/bluerov2_underwater.world
```

---

## 4. Software Architecture

### 4.1 Package Map (`duburi_ws/src/`)

```
duburi_ws/src/
├── duburi_interfaces/     # Custom ROS2 msgs, srvs, actions
├── duburi_bringup/        # Launch files, param yamls, mode configs
├── duburi_driver/         # MAVLink driver node (pymavlink ↔ ROS2)
│                          #   owns Pixhawk connection, publishes telemetry,
│                          #   subscribes to control commands
├── duburi_control/        # PID control loops (depth, heading, position)
├── duburi_teleop/         # Joystick → RC override (manual piloting)
└── duburi_mission/        # State machine missions (YASMIN FSM)
```

### 4.2 Data Flow

```
[Joystick Joy msg]
       ↓
[duburi_teleop]  ──/duburi/rc_override──→ [duburi_driver]
                                                  │
[duburi_control] ──/duburi/attitude_cmd──→         │ pymavlink
[duburi_control] ──/duburi/depth_cmd────→         │ RC_CHANNELS_OVERRIDE
[duburi_mission] ──/duburi/mission_cmd──→         │ SET_ATTITUDE_TARGET
                                                  │ SET_POSITION_TARGET
                                          [Pixhawk / ArduSub SITL]
                                                  │
                                          AHRS2 / ATTITUDE / HEARTBEAT
                                                  │
                                       [duburi_driver publishes]
                                          /duburi/attitude  (RPY)
                                          /duburi/depth     (meters)
                                          /duburi/armed     (bool)
                                          /duburi/mode      (string)
                                          /duburi/battery   (voltage)
```

### 4.3 Node Responsibilities

| Node | Package | Owns |
|---|---|---|
| `driver_node` | duburi_driver | MAVLink connection, all sends/receives |
| `control_node` | duburi_control | PID loops for depth/heading/position |
| `teleop_node` | duburi_teleop | Joy → twist → RC override commands |
| `mission_node` | duburi_mission | FSM state machine, mission logic |

**Rule**: Only `driver_node` touches pymavlink. All other nodes talk to it via ROS2 topics/services.

---

## 5. MAVLink / ArduSub Patterns (Proven from Reference Codebases)

### 5.1 Connection (always do this first)

```python
import os
os.environ['MAVLINK20'] = '1'  # Enable MAVLink 2.0 (18 RC channels)
from pymavlink import mavutil

master = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
master.wait_heartbeat()
boot_time = time.time()
# After wait_heartbeat(), master.target_system and target_component are set
```

### 5.2 Arm / Disarm

```python
# Arm
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,  1, 0, 0, 0, 0, 0, 0)   # param1=1 → arm
master.motors_armed_wait()

# Disarm (switch to MANUAL first, then wait 3s)
_set_mode("MANUAL")
time.sleep(3)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,  0, 0, 0, 0, 0, 0, 0)   # param1=0 → disarm
master.motors_disarmed_wait()
```

### 5.3 Mode Switching

```python
def _set_mode(mode_name: str):
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    # Verify with COMMAND_ACK
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)

# ArduSub mode names (use these exact strings):
# MANUAL | STABILIZE | ALT_HOLD | POSHOLD | GUIDED | AUTO | SURFACE
```

### 5.4 RC Channel Override (direct thruster control)

```python
# Channel mapping (1-indexed, matches Duburi AUV convention):
# Ch1=Pitch  Ch2=Roll  Ch3=Throttle(depth)  Ch4=Yaw
# Ch5=Forward  Ch6=Lateral
# PWM: 1100–1900 µs | 1500=neutral | 65535=no override

def send_rc_override(pitch=1500, roll=1500, throttle=1500,
                     yaw=1500, forward=1500, lateral=1500):
    vals = [65535] * 18
    vals[0] = pitch
    vals[1] = roll
    vals[2] = throttle
    vals[3] = yaw
    vals[4] = forward
    vals[5] = lateral
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component, *vals)

# PWM conversion helper:
def pct_to_pwm(pct: float) -> int:
    """Convert -100..+100 percent to 1100..1900 PWM. 0→1500."""
    return int(1500 + (pct / 100.0) * 400)
```

### 5.5 Attitude Target (yaw/heading control)

```python
from pymavlink.quaternion import QuaternionBase
import math

def set_attitude(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)),
        master.target_system, master.target_component,
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        QuaternionBase([math.radians(a) for a in (roll_deg, pitch_deg, yaw_deg)]),
        0, 0, 0, 0)
```

### 5.6 Depth Setpoint (ALT_HOLD mode required)

```python
def set_depth(depth_m: float):
    """depth_m: negative = below surface. e.g. -0.5 = 0.5m deep"""
    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)),
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        (mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE),
        0, 0, depth_m,
        0, 0, 0, 0, 0, 0, 0, 0)
```

### 5.7 Reading Telemetry (AHRS2 — primary attitude source)

```python
def get_attitude():
    msg = master.recv_match(type='AHRS2', blocking=True, timeout=1.0)
    if not msg:
        return None
    yaw_deg = math.degrees(msg.yaw)
    if yaw_deg < 0:
        yaw_deg += 360.0
    depth_m = msg.altitude   # Negative when underwater
    return {'yaw': yaw_deg, 'roll': math.degrees(msg.roll),
            'pitch': math.degrees(msg.pitch), 'depth': depth_m}
```

### 5.8 Heartbeat (mandatory — send ≥ 1 Hz or failsafe triggers)

```python
def send_heartbeat():
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0)
```

---

## 6. PID Control — Design Philosophy (JSF-AV Inspired)

All PID controllers live in `duburi_control`. Each axis (depth, heading, position-x, position-y) has its own controller instance. Never share state between axes.

### 6.1 Tuned Starting Values (from 2023/2025 reference codebases)

| Axis | Kp | Ki | Kd | Output Range |
|---|---|---|---|---|
| Heading (yaw) | 1.0–2.0 | 0.0 | 0.1 | ±500 PWM from 1500 |
| Depth | 0.9 | 0.01 | 0.1 | PWM 1350–1650 |
| Position (DVL) | 0.7 | 0.0 | 0.05 | ±70 PWM delta |

### 6.2 Mandatory PID Features (robustness requirements)

```python
class PIDController:
    def __init__(self, kp, ki, kd, out_min, out_max, integral_max):
        ...
    # Required:
    # - Anti-windup: clamp integral when output saturates
    # - Derivative on measurement (not error) → no derivative kick on setpoint change
    # - Output clamping: hard limits on PWM output
    # - Reset: clear integral on mode change or re-arm
    # - Deadband: skip tiny corrections that cause thruster jitter
```

### 6.3 Heading Wrap-Around (always use this)

```python
def heading_error(target: float, current: float) -> float:
    """Shortest-path error on circular 0-360 space."""
    err = (target - current + 540) % 360 - 180
    return err
```

---

## 7. JSF-AV Principles Applied to AUV Control

JSF-AV (Joint Strike Fighter Coding Standards) adapted for our context:

| Principle | What it means here |
|---|---|
| **Single Entry/Exit** | Each control function has one clear responsibility |
| **No dynamic memory in loop** | No allocations inside ROS timer callbacks |
| **Bounded loops** | Every `while` loop has a timeout or counter limit |
| **Fail-safe defaults** | On any error → stop thrusters (RC neutral), surface |
| **Clear interfaces** | Nodes communicate only through defined ROS topics/services |
| **No global mutable state** | PID state lives in class instances, not module globals |
| **Defensive input validation** | Clamp all incoming PWM/setpoints at node boundary |
| **Deterministic timing** | Use ROS timers, not time.sleep() in control loops |

---

## 8. Custom Message Types (`duburi_interfaces`)

### Published by `driver_node`:
- `/duburi/attitude` — `duburi_interfaces/Attitude` (roll, pitch, yaw in degrees, depth in m)
- `/duburi/state` — `duburi_interfaces/VehicleState` (armed bool, mode string, battery_v float)

### Subscribed by `driver_node`:
- `/duburi/rc_override` — `duburi_interfaces/RCOverride` (6 channel PWM values)
- `/duburi/attitude_cmd` — `duburi_interfaces/AttitudeCmd` (roll, pitch, yaw setpoints)
- `/duburi/depth_cmd` — `std_msgs/Float32` (depth setpoint in meters, negative)

### Control internal:
- `/duburi/cmd_vel` — `geometry_msgs/Twist` (teleop velocity commands)

---

## 9. ArduSub Modes Reference

| Mode | ArduSub Name | Use Case |
|---|---|---|
| `MANUAL` | MANUAL | Raw RC override, arm/disarm |
| `STABILIZE` | STABILIZE | Attitude stabilized, pilot controls movement |
| `ALT_HOLD` | ALT_HOLD | Depth-hold autopilot, lateral still manual |
| `POSHOLD` | POSHOLD | Position hold (needs GPS/DVL) |
| `GUIDED` | GUIDED | Waypoint following from GCS |
| `AUTO` | AUTO | Pre-programmed mission |
| `SURFACE` | SURFACE | Emergency surface |

**Typical mission sequence**: `MANUAL` → arm → `ALT_HOLD` → mission

---

## 10. Reference Codebases (Study These)

| Location | Placement | Key Lessons |
|---|---|---|
| `Reference CodeBase/2023/` | **2nd World 2023** | Core pymavlink patterns, heading PID, depth PID |
| `Reference CodeBase/Robosub-2025-Duburi/` | **8th World 2025** | Full ROS2 architecture, YASMIN FSM, DVL integration |
| `Reference CodeBase/ardusub-interface/` | BumblebeeAS research | ROS2 + behavior trees, setpoint-based control |
| `Reference CodeBase/BareMinimum/` | Internal tool | Minimal working pymavlink, good for reference |
| `Reference CodeBase/marzan/` | Team member | Joy → ROS2 → pymavlink bridge pattern |
| `Reference CodeBase/mishu/` | Team member | Standalone mission scripts, time-based movements |

**The 2023 codebase is the ground truth** for proven MAVLink patterns. When in doubt, check `/2023/control/Duburi_5/BracU_Duburi_Control-main/utils.py`.

---

## 11. Development Workflow

### Step 1: Always sim-first
```bash
# Bring up sim (in docker terminals)
sim_vehicle.py -L RATBeach -v ArduSub -f vectored_6dof --model=JSON \
  --out=udp:0.0.0.0:14550 --out=udp:127.0.0.1:14551 --console
gz sim -v 3 -r src/bluerov2_gz/worlds/bluerov2_underwater.world

# Build and run
cd ~/Ros_workspaces/duburi_ws
colcon build --symlink-install
source install/setup.zsh
ros2 launch duburi_bringup sim.launch.py
```

### Step 2: Verify connectivity
```bash
ros2 topic echo /duburi/state      # Should show armed=false, mode=MANUAL
ros2 topic echo /duburi/attitude   # Should show attitude updating
```

### Step 3: Test control
```bash
ros2 service call /duburi/arm std_srvs/srv/SetBool "{data: true}"
ros2 topic pub /duburi/depth_cmd std_msgs/msg/Float32 "{data: -0.5}"
```

---

## 12. Environment & Paths

```bash
# Docker env
ROS_DOMAIN_ID=42
GZ_SIM_RESOURCE_PATH=~/Ros_workspaces/colcon_ws/src/bluerov2_gz/models:...
GZ_SIM_SYSTEM_PLUGIN_PATH=~/stuff/ardupilot_gazebo/build

# Workspaces
~/Ros_workspaces/colcon_ws   # bluerov2_gz sim (DO NOT MODIFY)
~/Ros_workspaces/duburi_ws   # OUR codebase (this workspace)

# Tools
~/stuff/ardupilot/            # ArduSub SITL
~/stuff/ardupilot_gazebo/     # Gazebo-ArduPilot bridge plugin
```

---

## 13. Safety Rules (Non-Negotiable)

1. **Always have a disarm mechanism** — keyboard interrupt, ROS shutdown, or timer
2. **Heartbeat must be maintained** — spawn a thread or timer for it; no loop can block it
3. **Neutral on startup** — all RC channels default to 1500 (not 65535) during mission
4. **Pool test checklist**: propellers clear? tether on? topside can see Jetson?
5. **Autonomous mission**: timer-delayed start (run code, wait N seconds, remove tether)
6. **DVL offset**: `dvl_depth_match = 0.78` (calibrated value from 2025 competition)

---

## 14. Context Files (in `.claude/context/`)

| File | Contents |
|---|---|
| `mavlink-reference.md` | All MAVLink messages, type masks, enums |
| `ardusub-reference.md` | ArduSub-specific parameters, modes, behaviors |
| `pid-theory.md` | PID design, anti-windup, tuning guide |
| `proven-patterns.md` | Complete code patterns from reference codebases |
| `sim-setup.md` | Detailed simulation bring-up guide |
| `mission-design.md` | FSM architecture, state machine patterns |
| `hardware-setup.md` | Pool setup, BlueOS, network topology details |
| `ros2-conventions.md` | ROS2 coding conventions for this project |
