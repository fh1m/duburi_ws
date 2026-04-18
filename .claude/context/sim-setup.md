# Simulation Setup Guide — Duburi AUV

Complete step-by-step guide to bring up the SITL + Gazebo simulation stack.
All commands run inside the auv-ros2 distrobox docker.

---

## Architecture Overview

```
[ArduSub SITL] ←JSON/UDP→ [ardupilot_gazebo plugin] ←→ [Gazebo Harmonic]
      ↕ UDP:14550
[Our ROS2 nodes (duburi_ws)]
      ↕
[MAVROS or direct pymavlink]
```

The ardupilot_gazebo plugin bridges physics: Gazebo simulates buoyancy, drag, and
thruster forces; the plugin converts these to JSON and feeds them to ArduSub SITL.
ArduSub runs its own EKF3 and PID loops, sending back motor commands to Gazebo.

---

## Step 1: Environment Check

```bash
# Verify paths are set (should be in ~/.zshrc already)
echo $GZ_SIM_RESOURCE_PATH
echo $GZ_SIM_SYSTEM_PLUGIN_PATH
# Should include ardupilot_gazebo/build and bluerov2_gz paths

# Verify ArduSub SITL is available
which sim_vehicle.py
# → ~/stuff/ardupilot/Tools/autotest/sim_vehicle.py

# Verify Gazebo plugin built
ls ~/stuff/ardupilot_gazebo/build/*.so
# → Should show libArduPilotPlugin.so or similar
```

---

## Step 2: Launch ArduSub SITL

```bash
# Terminal 1 (or tmux pane)
sim_vehicle.py \
  -L RATBeach \
  -v ArduSub \
  -f vectored_6dof \
  --model=JSON \
  --out=udp:0.0.0.0:14550 \
  --out=udp:127.0.0.1:14551 \
  --console

# Flags explained:
# -L RATBeach       : Start location (GPS coordinates for RATBeach test pool)
# -v ArduSub        : Vehicle type
# -f vectored_6dof  : BlueROV2 Heavy frame (8 thrusters, full 6DOF)
# --model=JSON      : Use JSON interface for Gazebo plugin
# --out UDP:14550   : Output to all interfaces:14550 (our ROS nodes connect here)
# --out UDP:14551   : Extra output for monitoring/MAVROS if needed
# --console         : Show MAVProxy text console

# Wait for: "APM: EKF3 IMU0 is using barometer" before proceeding
```

**Troubleshooting SITL:**
- If it hangs: kill and retry — sometimes SITL needs a fresh start
- If GPS warnings: normal for AUV (we don't use GPS)
- If EKF errors: wait 30 seconds, EKF3 needs time to initialize from cold start
- Port already in use: `pkill -f sim_vehicle` and retry

---

## Step 3: Launch Gazebo

```bash
# Terminal 2 (or tmux pane)
cd ~/Ros_workspaces/colcon_ws
gz sim -v 3 -r src/bluerov2_gz/worlds/bluerov2_underwater.world

# Flags:
# -v 3   : Verbosity level 3 (shows warnings, useful for debugging)
# -r     : Run immediately (don't pause at start)

# Available world files:
# bluerov2_underwater.world       → Standard BlueROV2 (vectored, 6 thrusters)
# bluerov2_heavy_underwater.world → Heavy config (vectored_6dof, 8 thrusters) ← USE THIS
# bluerov2_ping.world             → With sonar
```

**Wait for**: Gazebo window to open and vehicle to be visible underwater.
**Expected**: Vehicle should be positionally stable (slight float/sink is normal until SITL connects).

---

## Step 4: Verify SITL-Gazebo Connection

In the SITL console (MAVProxy), you should see:
```
APM: JSON interface: Packet received
APM: Barometer calibrated
APM: EKF3 IMU0 is using barometer
```

If Gazebo and SITL are NOT communicating:
- Check `GZ_SIM_SYSTEM_PLUGIN_PATH` includes ardupilot_gazebo/build
- Check `GZ_SIM_RESOURCE_PATH` includes bluerov2_gz paths
- Restart both Gazebo and SITL

---

## Step 5: Build and Run Our Nodes

```bash
# Terminal 3
cd ~/Ros_workspaces/duburi_ws
colcon build --symlink-install
source install/setup.zsh

# Run the driver node (connects to SITL)
ros2 launch duburi_bringup sim.launch.py
```

**Or run nodes manually for debugging:**
```bash
ros2 run duburi_driver driver_node --ros-args \
  -p connection_string:=udpin:0.0.0.0:14550 \
  -p mode:=sim
```

---

## Step 6: Verify Connectivity

```bash
# Check state topic
ros2 topic echo /duburi/state --once
# Expected: armed=false, mode=MANUAL, battery_v=16.x

# Check attitude topic
ros2 topic echo /duburi/attitude
# Expected: yaw updating, depth ~0.0 (at surface)

# Check topic list
ros2 topic list | grep duburi
```

---

## Step 7: Test Basic Commands

```bash
# Arm vehicle
ros2 service call /duburi/arm std_srvs/srv/SetBool "{data: true}"
# → Thrusters should spin briefly in Gazebo

# Set depth (requires ALT_HOLD mode first)
ros2 service call /duburi/set_mode std_srvs/srv/SetBool  # TODO: proper service
ros2 topic pub /duburi/depth_cmd std_msgs/msg/Float32 "{data: -0.5}" --once
# → Vehicle should descend to 50cm in Gazebo

# Disarm
ros2 service call /duburi/arm std_srvs/srv/SetBool "{data: false}"
```

---

## Sim-Specific Gotchas

### 1. Depth starts at 0 (surface)
SITL starts with vehicle at surface. Depth commands need ALT_HOLD mode first.
After arming in MANUAL, switch to ALT_HOLD, then send depth setpoint.

### 2. Vehicle may drift horizontally
SITL doesn't have GPS lock → EKF3 may drift in XY. For depth-only missions,
this is fine. For position hold, need to bridge Gazebo ground truth as fake GPS.

### 3. Thrusters in Gazebo respond instantly
Real thrusters have motor ramp-up time. Sim PID gains may need to be more
conservative on hardware (reduce Kp by ~30% as starting point).

### 4. AHRS2 altitude in SITL
In SITL, `AHRS2.altitude` reflects simulated depth from Gazebo physics.
It should match what you see in the Gazebo viewport.

### 5. RC override in MANUAL mode
In MANUAL mode, RC override directly controls thrusters.
In ALT_HOLD, ch3 (throttle) is relative: 1500=hold, >1500=rise, <1500=dive.

---

## Quick Restart Procedure

When something goes wrong and you need a fresh start:

```bash
# Kill everything
pkill -f sim_vehicle
pkill -f gz
pkill -f duburi

# Wait 3 seconds, then restart in order:
# 1. SITL (wait for "EKF3 ready")
# 2. Gazebo (wait for window)
# 3. Our nodes
```

---

## Tmux Session Layout (Recommended)

```
Window 0: SITL
Window 1: Gazebo
Window 2: ROS2 duburi nodes
Window 3: ros2 topic echo / debugging
Window 4: Mission execution
```

Or use the `gazebo_up` alias already in ~/.zshrc for Gazebo.

---

## MAVROS vs Direct Pymavlink in SIM

Two approaches available:

| | Direct Pymavlink | MAVROS |
|---|---|---|
| **Connection** | `udpin:0.0.0.0:14550` | `fcu_url: udp://:14551@localhost:14550` |
| **Used in** | Our driver_node (reference codebase pattern) | ardusub-interface reference |
| **Pros** | Proven, simple, direct | ROS2 native, services/actions |
| **Cons** | More boilerplate | MAVROS overhead, extra node |
| **Our choice** | ✅ Preferred | Available as fallback |

If using MAVROS alongside our node, point it at port 14551 (the second `--out` we added to SITL).
