# Hardware Setup & Pool Operations — Duburi AUV

Reference for physical setup, network configuration, and pool testing procedures.

---

## Physical Hardware

| Component | Model | Interface | IP/Port |
|---|---|---|---|
| Flight Controller | Pixhawk 2.4.8 | Serial (USB/UART) → BlueOS | Via MAVLink endpoint |
| Main SBC | Jetson Orin Nano Dev Kit | Ethernet switch | 192.168.2.69 |
| Companion | Raspberry Pi 4B 8GB | Ethernet switch | 192.168.2.1 |
| DVL | Nortek Nucleus1000 | Ethernet switch | 192.168.2.201 |
| Cameras | Blue Robotics Low-Light ×2 | USB to Jetson | /dev/v4l/by-id/... |
| ESCs | Blue Robotics Basic ESC ×8 | Pixhawk PWM | — |
| Thrusters | Blue Robotics T200 ×8 | Via ESCs | — |
| Depth Sensor | Bar30 | I2C to Pixhawk | Internal |

---

## Network Topology

```
                    [Tether] (copper + fiber)
                        ↕
              [Topside Laptop / Ground Station]
              192.168.2.xxx (same subnet via tether)
                        ↕
              ┌─────────────────────────────┐
              │   AUV Ethernet Switch       │
              │   192.168.2.x subnet        │
              └──┬────┬────┬───────────────┘
                 │    │    │
            [Jetson] [Pi] [DVL]
           .2.69  .2.1  .2.201
               Gateway: .2.2
```

**Accessing AUV from ground station:**
```bash
# SSH to Jetson
ssh user@192.168.2.69

# VNC / Remote Desktop to Jetson (for GUI)
# Use Remmina or any VNC client → 192.168.2.69:5900

# BlueOS web UI (monitoring, calibration, video)
# Browser → http://192.168.2.1

# MAVProxy on ground station
mavproxy.py --master=udpin:0.0.0.0:14550
```

---

## MAVLink Endpoint (BlueOS Config)

```
Name: inspector
Type: UDP Client
IP: 192.168.2.69    (Jetson — where our code runs)
Port: 14550

Flow: Pixhawk → USB → Pi (BlueOS) → UDP client → Jetson:14550
      Jetson: mavutil.mavlink_connection("udpin:0.0.0.0:14550")
```

This means:
- BlueOS acts as MAVLink router
- Pi SENDS UDP packets to Jetson (client mode)
- Jetson RECEIVES with udpin (server mode, binds to port)

**IMPORTANT**: When testing without BlueOS (direct Pixhawk to Jetson via USB):
```python
# Direct USB serial connection:
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

# Or via direct Ethernet if Pixhawk has Ethernet module:
master = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
```

---

## DVL — Nortek Nucleus1000

**Connection options:**
- TCP (primary): `192.168.2.201:9000`
- Serial: `/dev/ttyUSB0` at 115200 baud (backup)

**ROS2 driver**: `nucleus_driver_ros2` package (already in Reference CodeBase)

**Key packet types:**
- `0xD2` AHRS: heading, roll, pitch, depth (use for attitude backup)
- `0xDC` INS: absolute position x/y/z, velocity
- `0xB4` Bottom Track: velocity relative to bottom (distance traveled)
- `0xBE` Water Track: water-relative velocity
- `0xAA` Altimeter: distance to seafloor

**Calibration**: `dvl_depth_match = 0.78` offset from DVL depth to true depth (measured Robosub 2025).

**Connecting via ROS2 service:**
```bash
ros2 service call /nucleus_node/connect_tcp \
  nucleus_interfaces/srv/ConnectTcp "{host: '192.168.2.201', port: 9000}"
ros2 service call /nucleus_node/start std_srvs/srv/Trigger
```

---

## Camera Setup

```python
# Forward camera (GStreamer pipeline — from 2025 codebase)
CAMERA_FRONT_PIPELINE = (
    "v4l2src device=/dev/v4l/by-id/"
    "usb-H264_USB_Camera_H264_USB_Camera_2020032801-video-index0 "
    "! video/x-raw, width=640, height=480, framerate=30/1 "
    "! videoconvert ! video/x-raw, format=BGR ! appsink"
)

# Downward camera
CAMERA_DOWN_PIPELINE = (
    "v4l2src device=/dev/v4l/by-id/"
    "usb-Sonix_Technology_Co.__Ltd._exploreHD_USB_Camera_SN00009-video-index0 "
    "! video/x-raw, width=640, height=480, framerate=30/1 "
    "! videoconvert ! video/x-raw, format=BGR ! appsink"
)

# OpenCV usage:
import cv2
cap = cv2.VideoCapture(CAMERA_FRONT_PIPELINE, cv2.CAP_GSTREAMER)
```

---

## GPIO (Jetson Orin Nano) — Torpedo / Dropper

```python
import Jetson.GPIO as GPIO

# Pins used (from 2025 reference):
# Pin 11: Torpedo 1
# Pin 13: Torpedo 2
# Pin 15: Dropper 1
# Pin 19: Dropper 2

GPIO.setmode(GPIO.BOARD)

def fire_actuator(pin: int, pulse_ms: float = 100):
    """Fire torpedo or dropper actuator with a pulse."""
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.output(pin, GPIO.LOW)    # Activate (active low)
    time.sleep(pulse_ms / 1000)
    GPIO.output(pin, GPIO.HIGH)   # Release

def cleanup_gpio():
    GPIO.cleanup()
```

---

## Pool Testing Procedure

### Pre-dive Checklist

```
□ O-rings greased and seated properly
□ All penetrators hand-tight
□ Battery charged (>90%)
□ Tether secured to AUV
□ Jetson booted (can SSH in)
□ All cameras visible via BlueOS
□ MAVLink connected (BlueOS shows "connected")
□ ArduSub parameters correct (check ARMING_CHECK, motor mapping)
□ Test arm/disarm via QGC or cockpit
□ Test each thruster individually (low PWM, 10 seconds)
□ DVL connected and sending packets
□ Code deployed to Jetson
□ Run code → verify /duburi/state shows connected
□ Test depth hold at 0.3m (brief)
□ Test heading hold (rotate 90° left, 90° right)
```

### Autonomous Mission Procedure

```bash
# On Jetson (via SSH):
cd ~/Ros_workspaces/duburi_ws
source install/setup.zsh

# Launch with delayed start
ros2 launch duburi_bringup mission.launch.py mode:=pool

# Output should show: "Mission starting in 10 seconds... REMOVE TETHER NOW"
# Operator removes tether during countdown
# Mission begins after countdown
```

### Emergency Recovery

1. SSH to Jetson: `ssh user@192.168.2.69`
2. `ros2 service call /duburi/emergency_surface std_srvs/srv/Trigger`
3. OR from MAVProxy console: `mode surface`
4. Pull tether to guide AUV if needed

---

## Jetson Orin Nano Setup Notes

```bash
# Check GPU for YOLO inference
nvidia-smi

# Check CUDA availability in Python
python3 -c "import torch; print(torch.cuda.is_available())"

# Static IP (should already be configured):
# /etc/netplan/01-network.conf
# Address: 192.168.2.69/24, Gateway: 192.168.2.2, DNS: 8.8.8.8

# Allow ros2 traffic on domain 42
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

---

## BlueOS Tips

- **Access**: http://192.168.2.1 from any device on same subnet
- **ArduSub params**: Terminal → `param show` or web param editor
- **Video streams**: Video tab → manage camera streams
- **Log download**: Tools → Log download
- **Calibration**: Sensors tab → compass, accelerometer
- **Ping sonar**: Extensions → Ping Sonar (if installed)

**Checking MAVLink health in BlueOS:**
- MAVLink Inspector shows live message rates
- If AHRS2 rate is 0, something is wrong with ArduSub/Pixhawk

---

## Troubleshooting Common Issues

| Problem | Likely Cause | Fix |
|---|---|---|
| Can't ARM | Pre-arm check fails | Check ARMING_CHECK param, look at STATUSTEXT |
| Vehicle sinks immediately | Buoyancy misconfigured | Adjust ballast foam |
| Heading drifts in ALT_HOLD | Compass interference | Re-calibrate compass away from thrusters |
| Depth oscillates | PID gains too high | Reduce Kp for depth |
| Can't connect MAVLink | BlueOS endpoint misconfigured | Check IP in BlueOS → check 192.168.2.69 |
| DVL not reading | Wrong IP or TCP port | Ping 192.168.2.201, check port 9000 |
| Camera not found | Device path changed | Check `ls /dev/v4l/by-id/` |
| Node crashed | Missing Python dep | `pip install pymavlink` on Jetson |
