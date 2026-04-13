# Makis — Autonomous Mecanum Tank Robot

A fully autonomous/manually-operated mecanum-wheel tank built on **Raspberry Pi 5**,
running **ROS 2 Humble** inside Docker, with LiDAR-based **SLAM + Nav2** navigation,
an **Arduino Uno** motor controller, and an **Android** companion app.

---

## Hardware BOM

| Component | Description |
|-----------|-------------|
| Raspberry Pi 5 (4/8 GB) | Main compute |
| Arduino Uno | Motor + encoder low-level controller |
| Motor Servo Shield (PCA9685 + TB6612) | Motor driver shield with PS2 header |
| 4 × 37 mm DC Motor, 1:90 gear, encoder | Drive motors for mecanum wheels |
| 4 × Mecanum wheels (65 mm) | Omnidirectional drive |
| PS2 Wireless Controller + Receiver | Manual remote fallback |
| Raspberry Pi Camera v2 | Front-facing camera |
| LSM303DLHC breakout | Accelerometer + Magnetometer (I²C) |
| L3GD20 breakout | Gyroscope (I²C) |
| Hitachi HLS-LFCD2 LiDAR | 360° 2D laser scanner (USB) |
| LiPo 3S 2200 mAh | Main battery |
| 5V/5A BEC (or UBEC) | RPi 5 power |

---

## Software Stack

```
Android App (rosbridge WebSocket)
        │
        ▼
┌────────────────────────────────────────────────────────┐
│  Docker Container — ROS 2 Humble (ARM64)               │
│                                                        │
│  hls_lfcd_lds_driver  →  /scan                         │
│  v4l2_camera          →  /camera/image_raw             │
│  tank_imu             →  /imu/data  /imu/mag           │
│  tank_hardware        →  /odom  /cmd_vel               │
│  robot_localization   →  /odometry/filtered            │
│  slam_toolbox         →  /map  /tf                     │
│  nav2                 →  /goal_pose → /cmd_vel         │
│  rosbridge_suite      →  ws://rpi5:9090                │
│  web_video_server     →  http://rpi5:8080              │
└────────────────────────────────────────────────────────┘
        │ USB-Serial
        ▼
┌──────────────────────────────────┐
│  Arduino Uno                     │
│  Adafruit Motor Shield V2        │
│  PCA9685 (I²C 0x40) + 2×TB6612 │
│  4 encoders (INT0/INT1 + poll)   │
│  PS2 wireless receiver           │
└──────────────────────────────────┘
```

---

## Repository Layout

```
makis/
├── Dockerfile                     # ARM64 ROS 2 Humble image
├── docker-compose.yml             # Multi-service compose
├── docker/
│   └── entrypoint.sh
├── arduino/
│   └── tank_firmware/
│       └── tank_firmware.ino      # Complete Arduino firmware
├── ros2_ws/
│   └── src/
│       ├── tank_description/      # URDF + TF tree
│       ├── tank_hardware/         # Arduino bridge + odometry
│       ├── tank_imu/              # LSM303DLHC + L3GD20 driver
│       └── tank_bringup/          # Launch files + configs
├── android/                       # Android Studio project
└── docs/
    └── wiring_diagram.md
```

---

## Quick Start

### 1 — Flash Arduino
```bash
# Install Arduino libraries (Arduino IDE or arduino-cli)
# - Adafruit Motor Shield V2   (Library Manager)
# - PS2X_lib by Bill Porter    (Library Manager)
arduino-cli compile --upload -p /dev/ttyUSB0 arduino/tank_firmware/
```

### 2 — Build Docker image on RPi 5
```bash
# On Raspberry Pi 5
git clone https://github.com/yourname/makis
cd makis
docker compose build
docker compose up -d
```

### 3 — Start SLAM session
```bash
docker compose exec ros2 ros2 launch tank_bringup slam_nav.launch.py
```

### 4 — Android App
Open `android/` in Android Studio, update `RPI5_IP` in `strings.xml`, build & install.

---

## Mecanum Wheel Layout

```
  FRONT
[FL ╱]──────[FR ╲]
   │          │
   │   body   │
   │          │
[RL ╲]──────[RR ╱]
  REAR
```

Inverse kinematics (vx=fwd, vy=strafe, ω=yaw):
```
FL = vx − vy − ω·L
FR = vx + vy + ω·L
RL = vx + vy − ω·L
RR = vx − vy + ω·L
  where L = (wheelbase + trackwidth) / 2
```

---

## Serial Protocol (Arduino ↔ RPi)

| Direction | Format | Description |
|-----------|--------|-------------|
| RPi → Ard | `CMD:<FL>:<FR>:<RL>:<RR>\n` | Motor speeds −255…255 |
| Ard → RPi | `ENC:<fFL>:<fFR>:<fRL>:<fRR>:<dt_ms>\n` | Encoder ticks + Δt |
| Ard → RPi | `PS2:<lx>:<ly>:<rx>:<ry>:<btns>\n` | Joystick (0–255, bitmask) |
| Ard → RPi | `INFO:<msg>\n` | Status / error strings |

---

## ROS 2 Topic Map

| Topic | Type | Publisher |
|-------|------|-----------|
| `/scan` | `sensor_msgs/LaserScan` | hls_lfcd_lds_driver |
| `/camera/image_raw` | `sensor_msgs/Image` | v4l2_camera |
| `/imu/data` | `sensor_msgs/Imu` | tank_imu |
| `/imu/mag` | `sensor_msgs/MagneticField` | tank_imu |
| `/odom` | `nav_msgs/Odometry` | tank_hardware |
| `/odometry/filtered` | `nav_msgs/Odometry` | robot_localization |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox |
| `/cmd_vel` | `geometry_msgs/Twist` | nav2 / Android app |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Android app |

---

## License
MIT