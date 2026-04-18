# ROS2 Conventions — Duburi AUV Codebase

Coding standards, topic naming, package structure conventions for duburi_ws.
Follow these consistently across all packages.

---

## Topic Namespace

All our topics live under `/duburi/`:

```
/duburi/attitude          # VehicleAttitude: roll, pitch, yaw (deg), depth (m)
/duburi/state             # VehicleState: armed, mode, battery_v, connected
/duburi/rc_override       # RCOverride: 6 channel PWM values
/duburi/attitude_cmd      # AttitudeCmd: roll, pitch, yaw setpoints (deg)
/duburi/depth_cmd         # std_msgs/Float32: depth setpoint (m, negative)
/duburi/cmd_vel           # geometry_msgs/Twist: teleop velocity commands
/duburi/dvl/velocity      # DVL bottom-track velocity
/duburi/dvl/position      # DVL position estimate
/duburi/camera/front      # sensor_msgs/Image: forward camera
/duburi/camera/down       # sensor_msgs/Image: downward camera
/duburi/detection         # DetectionMsg: YOLO detection results
```

Services:
```
/duburi/arm               # std_srvs/SetBool: arm (true) / disarm (false)
/duburi/set_mode          # duburi_interfaces/SetMode: mode string
/duburi/emergency_surface # std_srvs/Trigger: immediate surface
/duburi/get_heading       # duburi_interfaces/GetHeading: current yaw
/duburi/get_depth         # duburi_interfaces/GetDepth: current depth
```

---

## Package Structure Templates

### duburi_interfaces (messages only, no code)

```
duburi_interfaces/
├── msg/
│   ├── Attitude.msg          # roll pitch yaw depth
│   ├── VehicleState.msg      # armed mode battery_v connected
│   ├── RCOverride.msg        # 6 channel PWM
│   ├── AttitudeCmd.msg       # roll pitch yaw setpoints
│   └── DetectionMsg.msg      # vision detection
├── srv/
│   ├── SetMode.srv           # string mode → bool success string message
│   ├── GetHeading.srv        # → float32 heading_deg
│   └── GetDepth.srv          # → float32 depth_m
├── CMakeLists.txt
└── package.xml
```

### duburi_driver (MAVLink bridge)

```
duburi_driver/
├── duburi_driver/
│   ├── __init__.py
│   ├── driver_node.py        # Main ROS2 node
│   ├── mavlink_client.py     # pymavlink wrapper class
│   └── conversions.py        # Unit conversions, angle normalization
├── config/
│   ├── sim.yaml              # Params for SIM mode
│   └── pool.yaml             # Params for POOL/HARDWARE mode
├── setup.py
└── package.xml
```

### duburi_control (PID controllers)

```
duburi_control/
├── duburi_control/
│   ├── __init__.py
│   ├── control_node.py       # ROS2 node — runs PID loops as timers
│   ├── pid_controller.py     # PIDController class
│   ├── heading_controller.py # Heading-specific (wrap-around logic)
│   └── depth_controller.py   # Depth-specific (DVL vs AHRS2 source)
├── config/
│   └── pid_params.yaml       # Tuned PID gains
├── setup.py
└── package.xml
```

---

## Node Template

```python
#!/usr/bin/env python3
"""
One-line description of what this node does.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # --- Parameters ---
        self.declare_parameter('param_name', 'default_value')
        self._param = self.get_parameter('param_name').value
        
        # --- Publishers ---
        self._my_pub = self.create_publisher(
            MsgType, '/duburi/topic', 10)
        
        # --- Subscribers ---
        self.create_subscription(
            MsgType, '/duburi/other_topic',
            self._callback, 10)
        
        # --- Services ---
        self.create_service(
            SrvType, '/duburi/service', self._service_cb)
        
        # --- Timers ---
        self.create_timer(0.1, self._control_loop)   # 10 Hz
        
        self.get_logger().info(f"Node started, param={self._param}")
    
    def _callback(self, msg):
        pass
    
    def _service_cb(self, request, response):
        response.success = True
        return response
    
    def _control_loop(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Package.xml Template (Python ROS2 package)

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>duburi_package_name</name>
  <version>0.1.0</version>
  <description>Brief description</description>
  <maintainer email="duburi@example.com">BRACU Duburi</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_srvs</depend>
  <depend>duburi_interfaces</depend>

  <buildtool_depend>ament_python</buildtool_depend>
  <test_depend>ament_pep8</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## Setup.py Template

```python
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'duburi_package_name'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BRACU Duburi',
    description='...',
    license='MIT',
    entry_points={
        'console_scripts': [
            'node_name = package_name.module:main',
        ],
    },
)
```

---

## Launch File Template

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Config file
    config_dir = get_package_share_directory('duburi_bringup')
    config_file = os.path.join(config_dir, 'config', 'sim.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim',
                              description='Operating mode: sim|pool|hardware'),
        
        Node(
            package='duburi_driver',
            executable='driver_node',
            name='duburi_driver',
            parameters=[config_file],
            output='screen',
        ),
        Node(
            package='duburi_control',
            executable='control_node',
            name='duburi_control',
            parameters=[config_file],
            output='screen',
        ),
    ])
```

---

## Naming Conventions

```
Packages:      duburi_<name>              duburi_driver, duburi_control
Nodes:         duburi_<name> (snake_case) duburi_driver, duburi_control
Topics:        /duburi/<name>             /duburi/attitude, /duburi/state
Services:      /duburi/<verb>_<noun>      /duburi/arm, /duburi/set_mode
Messages:      PascalCase                 VehicleState, RCOverride
Classes:       PascalCase                 DriverNode, PIDController
Functions:     snake_case                 send_rc_override, get_heading
Private:       _leading_underscore        _heartbeat_cb, _control_loop
Constants:     UPPER_SNAKE                PWM_NEUTRAL, DVL_DEPTH_MATCH
```

---

## QoS Profiles

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# For telemetry (best-effort, low latency)
TELEMETRY_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)

# For commands (reliable)
COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10)

# For state (reliable, latched for late subscribers)
STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)
```

---

## Building Quickly During Development

```bash
cd ~/Ros_workspaces/duburi_ws

# Full build
colcon build --symlink-install

# Build single package
colcon build --symlink-install --packages-select duburi_driver

# Build with cmake args (C++ packages)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source after build
source install/setup.zsh

# Check for errors before running
colcon test --packages-select duburi_driver
```

---

## Logging Style

```python
# Use node logger, not print()
self.get_logger().debug("Low-level detail")
self.get_logger().info("Normal operation")
self.get_logger().warn("Something unexpected but not fatal")
self.get_logger().error("Error that needs attention")
self.get_logger().fatal("About to crash/emergency")

# For one-off scripts (not nodes), use basic logging:
import logging
logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)
log.info("Connected to ArduSub")
```
