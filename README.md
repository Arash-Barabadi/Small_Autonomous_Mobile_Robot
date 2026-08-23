# Small Autonomous Mobile Robot

A low-cost differential-drive autonomous mobile robot (AMR) running **ROS 2** via
**micro-ROS** on an ESP32, with LiDAR SLAM and Nav2.

The first design challenge was budget. A teleoperated architecture on a Jetson Orin NX
is straightforward — the Jetson is a powerful platform for compiling and running
everything — but it is expensive. To cut cost, the main microcontroller is an **ESP32
(~€5)** instead of an ~€800 compute module. The ESP32's main limitation is that it has
no direct ROS 2 support, but it *does* support **micro-ROS**, so all firmware is
implemented within the micro-ROS framework.

[🎥 Watch the project demo on YouTube](https://www.youtube.com/watch?v=UUFdapDHFw0)

<img width="726" height="694" alt="Robot render" src="https://github.com/user-attachments/assets/61477df1-b32f-45a5-b045-df3a07960808" />

![Robot_1st](https://github.com/user-attachments/assets/4693b5db-3369-45c2-93c8-e64371168808)

![Robot_2nd](https://github.com/user-attachments/assets/46b4d3b1-096e-4664-ba26-10dafb0cfbc8)

---

## Hardware

| Component | Part |
|---|---|
| MCU | ESP32-S3-N16R8 (16 MB flash, 8 MB octal PSRAM) — see migration notes below |
| LiDAR | LD14P (UART, receive-only) |
| IMU | BNO085 (I²C) |
| Odometry | Quadrature wheel encoders |
| Drive | Dual DC motors via H-bridge (PWM + direction) |

The project began on a classic ESP32-WROOM-32 and was later migrated to the ESP32-S3;
both the migration and the resulting dual-core firmware are documented below.

---

## Networking (ESP32 ↔ ROS 2 PC)

This robot uses **micro-ROS over WiFi (UDP)**. The ESP32 joins a WiFi network and
exchanges ROS 2 messages with the **micro-ROS agent** running on the PC.

Agent install guide: https://cps.unileoben.ac.at/install-micro-ros-on-esp32/

### Architecture

- **ESP32 publishes:** `/scan`, `/imu/data`, `/wheel_ticks`, …
- **ESP32 subscribes:** `/cmd_vel`, …
- **PC runs:** `micro_ros_agent` (UDP on port 8888)
- **Transport:** WiFi UDP (`udp4`)

**Requirements:**
1. ESP32 and PC on the same network.
2. ESP32 must be able to reach the PC's IP.
3. Port 8888/UDP must not be blocked by a firewall.

### A — ESP32 (microcontroller) config

Find the PC IP on Ubuntu:

```bash
ip a
```

Use the `inet` address under the WiFi interface (usually `wlp…`). Example:

```bash
inet 10.126.78.187/24 brd 10.126.78.255 scope global dynamic noprefixroute wlp0s20f3
```

Set it in `main.cpp`:

```cpp
char ssid[]     = "YOUR_WIFI_NAME";
char password[] = "YOUR_WIFI_PASSWORD";
IPAddress agent_ip(10, 126, 78, 187);   // PC IP on the same WiFi
const uint16_t agent_port = 8888;
```

### B — PC side: start the micro-ROS agent

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Verify:

```bash
ros2 topic list
ros2 topic echo /scan --once
```

---

## TF

<img width="863" height="939" alt="TF tree" src="https://github.com/user-attachments/assets/42e45042-70ee-4584-9e41-9056bdc7d7cb" />

### System overview

**Sensors & sources**
- Wheel encoders → wheel odometry node → `/odom` + `odom → base_link` TF
- LD14P LiDAR → micro-ROS → `/scan` (frame_id: `base_laser`)

**Static TF**
- `base_link → base_laser`
- `base_link → imu_link`

### SLAM
- Package: `slam_toolbox`
- Mode: `online_async`
- Frames: `map → odom → base_link → base_laser`

---

## Terminal commands (Ubuntu)

**1 — Start the micro-ROS agent (UDP over WiFi)**

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**2 — Teleoperation.** Drive with the standard ROS 2 keyboard teleop, which publishes
`geometry_msgs/Twist` on `/cmd_vel`. The robot subscribes to `/cmd_vel` and moves
accordingly.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**3 — Static transforms**

Base → LiDAR (meters, radians):

```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.04125 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id base_laser
```

Base → IMU (meters, radians):

```bash
ros2 run tf2_ros static_transform_publisher --x -0.0674 --y 0.006355 --z -0.0053 --roll 0 --pitch 0 --yaw -1.57 --frame-id base_link --child-frame-id imu_link
```

**4 — Wheel odometry node**

```bash
ros2 run wheel_odometry wheel_odometry_node
```

**5 — SLAM (slam_toolbox)**

```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/$USER/ros2_ws/src/wheel_odometry/config/slam_params.yaml
```

**6 — RViz**

```bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**7 — Navigation**

```bash
# ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=/home/$USER/ros2_ws/src/wheel_odometry/config/nav2_params.yaml
ros2 launch wheel_odometry navigation.launch.py map:=/home/arash/ros2_ws/maps/test_map.yaml

ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]
  }
}"
```

---

## ESP32-S3 migration (dual-core)

The firmware was ported from the ESP32-WROOM-32 to an **ESP32-S3-N16R8**. The S3's
second CPU core is used to separate real-time control from sensor I/O.

**Architecture**
- **Core 1 (APP_CPU):** `controlTask` — micro-ROS executor (`/cmd_vel` in, IMU + ticks
  out) and the motor loop, ~500 Hz.
- **Core 0 (PRO_CPU):** `lidarTask` — LD14P serial parsing and `/scan` assembly.
- micro-ROS is **not thread-safe across cores**, so every `rcl_publish` goes through a
  single FreeRTOS mutex (`publishGuarded()`).

**Pin map (S3)** — GPIO 34/35 don't exist on the S3, so encoders were moved:

| Signal | GPIO | | Signal | GPIO |
|---|---|---|---|---|
| R_IN1 | 4 | | ENC_L_A | 17 |
| R_IN2 | 5 | | ENC_L_B | 18 |
| R_PWM | 6 | | ENC_R_A | 8 |
| L_IN1 | 7 | | ENC_R_B | 9 |
| L_IN2 | 15 | | LIDAR_RX | 10 |
| L_PWM | 16 | | IMU_SDA / SCL | 12 / 13 |

**Notes**
- Motor PWM uses the LEDC v3 pin-based API (`ledcAttach(pin, …)` / `ledcWrite(pin, …)`),
  not the old channel-based API.
- LiDAR UART is **receive-only** (`Serial1.begin(230400, SERIAL_8N1, LIDAR_RX, -1)`) —
  the ESP never transmits to the LiDAR.
- Encoder ISRs use `ARDUINO_ISR_ATTR`, and `attachInterrupt()` is called at the **end**
  of `setup()` (after micro-ROS init). Attaching interrupts earlier during startup
  tripped the interrupt allocator on core 1 (a `gpio.c` / `intr_alloc.c` panic).

---

## Build & flash

Built with **PlatformIO** (on VS Code), a stronger alternative to the Arduino IDE.

```bash
# Compile
pio run

# Compile, flash, and open the serial monitor in one step (avoids port-busy conflicts)
pio run -t upload -t monitor
```

The S3 uses its **native USB** port (`/dev/ttyACM*`). Run the monitor from the project
folder so it reads `monitor_speed = 115200` from `platformio.ini`. If upload fails with a
port-busy error, close any open monitor first (`pkill -f "device monitor"`).

### Troubleshooting: micro-ROS build fails with `/bin/sh: syntax error near ';'`

pioarduino ships a Matter/CHIP build flag
(`CHIP_ADDRESS_RESOLVE_IMPL_INCLUDE_HEADER=<...>`) that CMake expands with literal
semicolons into the compile line, which corrupts every micro-ROS (microcdr) compile.
The fix removes the flag at its **source** (the generator), so it survives PlatformIO
regenerating the toolchain file each build:

```bash
sed -i 's|.*CHIP_ADDRESS_RESOLVE_IMPL_INCLUDE_HEADER.*|        # (removed Matter CHIP flag - breaks micro-ROS build)|' \
  ~/.platformio/packages/framework-arduinoespressif32-libs/esp32s3/pioarduino-build.py
```

Then rebuild clean:

```bash
rm -rf .pio
pio run
```

> ⚠️ If PlatformIO ever reinstalls `framework-arduinoespressif32-libs`, the flag returns
> and the build breaks again — just re-run the `sed` command above. (The official
> micro-ROS Docker image is a permanent alternative that avoids this class of bug.)

---

## Challenges accomplished

### Pure-rotation fix

During testing, the robot could move forward and turn while moving, but it could not
rotate in place with a pure angular command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 2.0}}" -r 5
```

Rotation worked as soon as a tiny forward velocity was added:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.02}, angular: {z: 2.0}}" -r 5
```

This showed the issue wasn't ROS 2 communication but drivetrain behavior: pure in-place
rotation wasn't overcoming static friction.

**Fix** — a small internal linear assist inside `cmdVelCallback()` for pure-rotation
commands, applied before deadzone compensation:

```cpp
if (fabsf(v) < 0.001f && fabsf(w) > 0.0f) {
  v = ROTATION_ASSIST_LINEAR;
}
v = compensateLinearDeadzone(v);
w = compensateAngularDeadzone(w);
```

with:

```cpp
static constexpr float ROTATION_ASSIST_LINEAR = 0.02f;
```

**Result:** pure rotation then worked reliably.

### Command-timeout watchdog

Originally the robot kept moving indefinitely if no new `/cmd_vel` arrived (e.g. WiFi
dropped or teleop stopped) — the last velocity stayed active and the robot kept going
uncontrolled. A command-timeout watchdog now checks the time since the last command and
stops both motors if none arrives within the timeout:

```cpp
// Update timestamp whenever a command arrives
void cmdVelCallback(const void *msgin) {
  last_cmd_us = micros();  // current time in microseconds

  const auto *msg = (const geometry_msgs__msg__Twist *)msgin;
  float v = constrain(msg->linear.x  / MAX_V, -1.0f, 1.0f);
  float w = constrain(msg->angular.z / MAX_W, -1.0f, 1.0f);

  float left_norm  = (v - w * TRACK / 2.0f) * K_LEFT;
  float right_norm = (v + w * TRACK / 2.0f) * K_RIGHT;

  setMotor(L_IN1, L_IN2, 0, speedToPwm(left_norm));
  setMotor(R_IN1, R_IN2, 1, speedToPwm(right_norm));
}

// Check timeout each loop
void checkCmdTimeout() {
  uint32_t now = micros();
  if (last_cmd_us && (now - last_cmd_us > CMD_TIMEOUT_US)) {
    safetyStop();
  }
}
```

### LiDAR coordinate-frame correction (LD14P → ROS)

The LD14P outputs scan data using a **clockwise, left-handed** convention, which does not
match the ROS REP-103 standard (**counter-clockwise, right-handed**). This caused mirrored
scans and wrong visualization in RViz and SLAM. Rather than compensating with TF, the
angle interpretation is corrected directly in `main.cpp` on the ESP32: each raw LD14 angle
is converted with θ_ros = −θ_ld14, and beam indexing/angle range use `[-π, π]`, so the
LiDAR behaves like a standard ROS sensor while TF only represents its physical mounting.

```cpp
// Full ROS-style range
static constexpr float ANGLE_MIN_RAD = -PI;
static constexpr float ANGLE_MAX_RAD =  PI;
static constexpr float ANGLE_INC_RAD = (2 * PI) / SCAN_BEAMS;

// Angle conversion: clockwise → counter-clockwise, left-handed → right-handed
static inline float ld14DegToRosRad(float angle_deg_ld14)
{
    float angle_rad_ld14 = angle_deg_ld14 * DEG2RAD;
    float angle_rad_ros  = -angle_rad_ld14;
    return wrapToPi(angle_rad_ros);
}

// Inside lidarSpinOnce(): convert each raw point before assigning it to a beam
float angle_rad_ros = ld14DegToRosRad(angle_deg_ld14);
float beam_f = (angle_rad_ros - ANGLE_MIN_RAD) / ANGLE_INC_RAD;
int   beam   = lroundf(beam_f);

// Unobserved beams are filled with INFINITY
scan_ranges[i] = INFINITY;
```

---

## References

- [Prof. Dr.-Ing. Lars N. Josler — LinkedIn post](https://www.linkedin.com/posts/prof-dr-ing-lars-n-josler-b35999161_robotics-engineering-opensource-share-7448631464512032768-y-hW)
- [micro-ROS agent install guide](https://cps.unileoben.ac.at/install-micro-ros-on-esp32/)
