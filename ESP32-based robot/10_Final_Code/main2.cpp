// ============================= Standard and micro-ROS Libraries =============================
#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <rmw_microros/rmw_microros.h>

// ============================= ROS 2 Libraries =============================
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <builtin_interfaces/msg/time.h>

#ifndef INFINITY
#define INFINITY (1.0f / 0.0f)
#endif

// ============================= micro-ROS Agent IP and Port =============================
char ssid[]     = "FRITZ!Box 7560 IQ";
char password[] = "28715329684835834345";
IPAddress agent_ip(192, 168, 178, 164);
const uint16_t agent_port = 8888;

// ============================= Motor Driver Pin Definitions =============================
const int R_IN1 = 19;
const int R_IN2 = 18;
const int R_PWM = 17;

const int L_IN1 = 27;
const int L_IN2 = 26;
const int L_PWM = 14;

// ============================== Watchdog variables =====================================
static uint32_t last_cmd_us = 0;
static constexpr uint32_t CMD_TIMEOUT_US = 500000; // 500 ms

// ============================= Encoder Pin Definitions =============================
const int ENC_L_A = 34;
const int ENC_L_B = 32;
const int ENC_R_A = 35;
const int ENC_R_B = 33;

static constexpr bool INVERT_LEFT_DIR  = false;
static constexpr bool INVERT_RIGHT_DIR = false;

// ============================= Drive tuning ============================================
// Physical robot parameters
static constexpr float TRACK = 0.163f;   // wheel separation (m)
static constexpr float MAX_V = 0.50f;    // max linear command (m/s)
static constexpr float MAX_W = 2.00f;    // max angular command (rad/s)

// Robot-level command deadzone compensation
// Keep these moderate. Startup/hold PWM below is the main fix.
static constexpr float MIN_EFFECTIVE_LINEAR  = 0.12f;
static constexpr float MIN_EFFECTIVE_ANGULAR = 0.18f;

// Side scaling
static constexpr float K_LEFT  = 1.00f;
static constexpr float K_RIGHT = 1.00f;

// Wheel-side minimum PWM to overcome friction
static constexpr uint8_t MIN_DUTY_L_FWD = 55;
static constexpr uint8_t MIN_DUTY_L_REV = 100;
static constexpr uint8_t MIN_DUTY_R_FWD = 55;
static constexpr uint8_t MIN_DUTY_R_REV = 55;

// Very small command deadband
static constexpr float CMD_DEADBAND_LINEAR  = 0.01f;
static constexpr float CMD_DEADBAND_ANGULAR = 0.03f;

// PWM slew limiting
static constexpr int16_t MAX_PWM_STEP = 10;

// Startup boost / hold minimum to remove hesitation
static constexpr uint32_t START_BOOST_US = 120000; // 120 ms
static constexpr int16_t START_BOOST_L = 140;
static constexpr int16_t START_BOOST_R = 140;
static constexpr int16_t HOLD_MIN_L = 95;
static constexpr int16_t HOLD_MIN_R = 95;

// ============================= LiDAR (LD14P) =============================================
static constexpr int LIDAR_RX_PIN = 16;
static constexpr int LIDAR_TX_PIN = 4;

static constexpr uint8_t LIDAR_HEADER            = 0x54;
static constexpr int     LIDAR_POINTS_PER_PACKET = 12;

static constexpr int   SCAN_BEAMS   = 720;
static constexpr float PI_F         = 3.14159265f;
static constexpr float TWO_PI_F     = 2.0f * PI_F;
static constexpr float DEG2RAD_F    = PI_F / 180.0f;

static constexpr float ANGLE_MIN_RAD = -PI_F;
static constexpr float ANGLE_INC_RAD = TWO_PI_F / SCAN_BEAMS;
static constexpr float ANGLE_MAX_RAD = ANGLE_MIN_RAD + ANGLE_INC_RAD * (SCAN_BEAMS - 1);

// ============================= BNO085 IMU ================================================
#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t bno_sensor_value;

static constexpr uint32_t BNO_REPORT_US = 10000;  // 100 Hz
static constexpr sh2_SensorId_t ORI_REPORT = SH2_ROTATION_VECTOR;

struct BnoImuData {
  float qx = 0.0f;
  float qy = 0.0f;
  float qz = 0.0f;
  float qw = 1.0f;

  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;

  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
};

BnoImuData latest_imu;

// ============================= micro-ROS Entities =======================================
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

rclc_executor_t executor;
rclc_support_t  support;
rcl_node_t      node;
rcl_allocator_t allocator;

// IMU publisher + timer
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;
rcl_timer_t imu_timer;

// Encoder ticks publisher + timer
rcl_publisher_t ticks_pub;
std_msgs__msg__Int32MultiArray ticks_msg;
rcl_timer_t ticks_timer;
static int32_t wheel_ticks_buf[2] = {0, 0};

// LiDAR LaserScan publisher
rcl_publisher_t scan_pub;
sensor_msgs__msg__LaserScan scan_msg;
static float scan_ranges[SCAN_BEAMS];
static float scan_intensities[SCAN_BEAMS];
static bool  scan_valid[SCAN_BEAMS];
static bool  lidar_first_frame    = true;
static float lidar_last_start_deg = 0.0f;
static int   lidar_frames_in_scan = 0;

// ============================ Error Checking Macros ======================================
#define RCCHECK(fn)     { if ((fn) != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t _rc = (fn); (void)_rc; }

// ============================= Runtime motor state =======================================
static int16_t current_left_pwm   = 0;
static int16_t current_right_pwm  = 0;
static int16_t target_left_pwm    = 0;
static int16_t target_right_pwm   = 0;

static bool drive_active = false;
static uint32_t drive_boost_until_us = 0;

// ============================= Encoder counters ==========================================
volatile int32_t left_ticks  = 0;
volatile int32_t right_ticks = 0;

// ============================= Helper: sign ==============================================
static inline float signf_local(float x) {
  if (x > 0.0f) return 1.0f;
  if (x < 0.0f) return -1.0f;
  return 0.0f;
}

// ============================= Helper: stamp with ROS time ===============================
static inline void stamp_now(builtin_interfaces__msg__Time *t)
{
  int64_t now_ns = rmw_uros_epoch_nanos();

  if (now_ns <= 0) {
    t->sec     = 0;
    t->nanosec = 0;
  } else {
    t->sec     = (int32_t)(now_ns / 1000000000LL);
    t->nanosec = (uint32_t)(now_ns % 1000000000LL);
  }
}

// ============================= LiDAR angle helpers =======================================
static inline float wrapToPi(float a)
{
  while (a >= PI_F)  a -= TWO_PI_F;
  while (a < -PI_F)  a += TWO_PI_F;
  return a;
}

static inline float ld14DegToRosRad(float angle_deg_ld14)
{
  float angle_rad_ld14 = angle_deg_ld14 * DEG2RAD_F;
  float angle_rad_ros  = -angle_rad_ld14;
  return wrapToPi(angle_rad_ros);
}

// ================================= Motor drive ==========================================
void setMotorRaw(int in1, int in2, int pwm_chan, int16_t speed_pwm, bool brake_on_zero) {
  uint8_t duty = (uint8_t)constrain((int)abs(speed_pwm), 0, 255);

  if (speed_pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed_pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    if (brake_on_zero) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, HIGH);
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }
  }

  ledcWrite(pwm_chan, duty);
}

static inline int16_t slewStep(int16_t current, int16_t target, int16_t max_step) {
  if (target > current + max_step) return current + max_step;
  if (target < current - max_step) return current - max_step;
  return target;
}

static inline int16_t applyHoldMinimum(int16_t pwm, int16_t hold_min) {
  if (pwm == 0) return 0;
  if (abs(pwm) < hold_min) {
    return (pwm > 0) ? hold_min : -hold_min;
  }
  return pwm;
}

static inline void applyMotorPwms(bool brake_on_zero = false) {
  uint32_t now_us = micros();

  bool want_motion = (target_left_pwm != 0 || target_right_pwm != 0);
  bool currently_stopped = (current_left_pwm == 0 && current_right_pwm == 0);

  if (want_motion && currently_stopped && !drive_active) {
    drive_active = true;
    drive_boost_until_us = now_us + START_BOOST_US;
  }

  if (!want_motion) {
    drive_active = false;
  }

  int16_t commanded_left  = target_left_pwm;
  int16_t commanded_right = target_right_pwm;

  // Startup boost
  if (drive_active && (int32_t)(drive_boost_until_us - now_us) > 0) {
    if (commanded_left != 0) {
      commanded_left = (commanded_left > 0) ? START_BOOST_L : -START_BOOST_L;
    }
    if (commanded_right != 0) {
      commanded_right = (commanded_right > 0) ? START_BOOST_R : -START_BOOST_R;
    }
  } else {
    // Hold minimum once moving
    commanded_left  = applyHoldMinimum(commanded_left, HOLD_MIN_L);
    commanded_right = applyHoldMinimum(commanded_right, HOLD_MIN_R);
  }

  current_left_pwm  = slewStep(current_left_pwm,  commanded_left,  MAX_PWM_STEP);
  current_right_pwm = slewStep(current_right_pwm, commanded_right, MAX_PWM_STEP);

  setMotorRaw(L_IN1, L_IN2, 0, current_left_pwm,  brake_on_zero);
  setMotorRaw(R_IN1, R_IN2, 1, current_right_pwm, brake_on_zero);
}

static inline void safetyStop() {
  target_left_pwm   = 0;
  target_right_pwm  = 0;
  current_left_pwm  = 0;
  current_right_pwm = 0;
  drive_active = false;
  drive_boost_until_us = 0;
  setMotorRaw(L_IN1, L_IN2, 0, 0, true);
  setMotorRaw(R_IN1, R_IN2, 1, 0, true);
}

void error_loop() {
  while (true) {
    safetyStop();
    delay(100);
  }
}

// Map normalized wheel demand [-1,1] to PWM with minimum duty compensation
static inline int16_t speedToPwmSide(float s_norm, bool is_left) {
  s_norm = constrain(s_norm, -1.0f, 1.0f);

  if (fabsf(s_norm) < 0.001f) {
    return 0;
  }

  uint8_t min_duty;
  if (is_left) {
    min_duty = (s_norm >= 0.0f) ? MIN_DUTY_L_FWD : MIN_DUTY_L_REV;
  } else {
    min_duty = (s_norm >= 0.0f) ? MIN_DUTY_R_FWD : MIN_DUTY_R_REV;
  }

  float mag = fabsf(s_norm);
  uint8_t duty = (uint8_t)(min_duty + mag * (255 - min_duty));

  return (s_norm >= 0.0f) ? (int16_t)duty : -(int16_t)duty;
}

// Deadzone compensation on commanded robot motion
static inline float compensateLinearDeadzone(float v) {
  if (fabsf(v) < CMD_DEADBAND_LINEAR) return 0.0f;
  if (fabsf(v) < MIN_EFFECTIVE_LINEAR) return signf_local(v) * MIN_EFFECTIVE_LINEAR;
  return v;
}

static inline float compensateAngularDeadzone(float w) {
  if (fabsf(w) < CMD_DEADBAND_ANGULAR) return 0.0f;
  if (fabsf(w) < MIN_EFFECTIVE_ANGULAR) return signf_local(w) * MIN_EFFECTIVE_ANGULAR;
  return w;
}

// ================================= cmd_vel callback =====================================
void cmdVelCallback(const void *msgin) {
  last_cmd_us = micros();

  const auto *msg = (const geometry_msgs__msg__Twist *)msgin;

  float v = constrain(msg->linear.x,  -MAX_V, MAX_V);
  float w = constrain(msg->angular.z, -MAX_W, MAX_W);

  v = compensateLinearDeadzone(v);
  w = compensateAngularDeadzone(w);

  float left_mps  = (v - w * TRACK / 2.0f) * K_LEFT;
  float right_mps = (v + w * TRACK / 2.0f) * K_RIGHT;

  float left_norm  = constrain(left_mps  / MAX_V, -1.0f, 1.0f);
  float right_norm = constrain(right_mps / MAX_V, -1.0f, 1.0f);

  target_left_pwm  = speedToPwmSide(left_norm,  true);
  target_right_pwm = speedToPwmSide(right_norm, false);

  Serial.print("cmd_raw v=");
  Serial.print(msg->linear.x, 4);
  Serial.print(" w=");
  Serial.print(msg->angular.z, 4);
  Serial.print(" | cmd_used v=");
  Serial.print(v, 4);
  Serial.print(" w=");
  Serial.print(w, 4);
  Serial.print(" | target_left_pwm=");
  Serial.print(target_left_pwm);
  Serial.print(" target_right_pwm=");
  Serial.println(target_right_pwm);
}

void checkCmdTimeout() {
  uint32_t now = micros();
  if (last_cmd_us != 0 && (uint32_t)(now - last_cmd_us) > CMD_TIMEOUT_US) {
    safetyStop();
    last_cmd_us = 0;
  }
}

// ============================= BNO085 helpers ============================================
bool bnoEnableReports() {
  bool ok = true;
  ok &= bno08x.enableReport(ORI_REPORT, BNO_REPORT_US);
  ok &= bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, BNO_REPORT_US);
  ok &= bno08x.enableReport(SH2_LINEAR_ACCELERATION, BNO_REPORT_US);
  return ok;
}

bool bnoInit() {
  if (!bno08x.begin_I2C()) {
    return false;
  }
  delay(100);
  return bnoEnableReports();
}

void bnoPoll() {
  while (bno08x.getSensorEvent(&bno_sensor_value)) {
    switch (bno_sensor_value.sensorId) {

      case SH2_ROTATION_VECTOR:
      case SH2_GAME_ROTATION_VECTOR:
        latest_imu.qx = bno_sensor_value.un.rotationVector.i;
        latest_imu.qy = bno_sensor_value.un.rotationVector.j;
        latest_imu.qz = bno_sensor_value.un.rotationVector.k;
        latest_imu.qw = bno_sensor_value.un.rotationVector.real;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        latest_imu.gx = bno_sensor_value.un.gyroscope.x;
        latest_imu.gy = bno_sensor_value.un.gyroscope.y;
        latest_imu.gz = bno_sensor_value.un.gyroscope.z;
        break;

      case SH2_LINEAR_ACCELERATION:
        latest_imu.ax = bno_sensor_value.un.linearAcceleration.x;
        latest_imu.ay = bno_sensor_value.un.linearAcceleration.y;
        latest_imu.az = bno_sensor_value.un.linearAcceleration.z;
        break;

      default:
        break;
    }
  }
}

void imuTimerCallback(rcl_timer_t*, int64_t) {
  bnoPoll();

  stamp_now(&imu_msg.header.stamp);

  imu_msg.orientation.x = latest_imu.qx;
  imu_msg.orientation.y = latest_imu.qy;
  imu_msg.orientation.z = latest_imu.qz;
  imu_msg.orientation.w = latest_imu.qw;

  imu_msg.angular_velocity.x = latest_imu.gx;
  imu_msg.angular_velocity.y = latest_imu.gy;
  imu_msg.angular_velocity.z = latest_imu.gz;

  imu_msg.linear_acceleration.x = latest_imu.ax;
  imu_msg.linear_acceleration.y = latest_imu.ay;
  imu_msg.linear_acceleration.z = latest_imu.az;

  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = 0.0;
    imu_msg.angular_velocity_covariance[i] = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }

  imu_msg.orientation_covariance[0] = 0.02;
  imu_msg.orientation_covariance[4] = 0.02;
  imu_msg.orientation_covariance[8] = 0.05;

  imu_msg.angular_velocity_covariance[0] = 0.01;
  imu_msg.angular_velocity_covariance[4] = 0.01;
  imu_msg.angular_velocity_covariance[8] = 0.01;

  imu_msg.linear_acceleration_covariance[0] = 0.10;
  imu_msg.linear_acceleration_covariance[4] = 0.10;
  imu_msg.linear_acceleration_covariance[8] = 0.10;

  RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
}

// ============================= LD14P minimal parser =====================================
struct __attribute__((packed)) LidarPoint {
  uint16_t distance;
  uint8_t  intensity;
};

struct __attribute__((packed)) LidarFrame {
  uint8_t  header;
  uint8_t  ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPoint point[LIDAR_POINTS_PER_PACKET];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t  crc8;
};

bool lidarReadFrame(LidarFrame &frame) {
  while (Serial2.available() >= (int)sizeof(LidarFrame)) {
    int c = Serial2.read();
    if (c != LIDAR_HEADER) {
      continue;
    }
    frame.header = (uint8_t)c;

    size_t want = sizeof(LidarFrame) - 1;
    size_t got  = Serial2.readBytes(((uint8_t*)&frame) + 1, want);
    if (got != want) return false;

    if (frame.ver_len != 0x2C) continue;
    return true;
  }
  return false;
}

void lidarSpinOnce() {
  LidarFrame f;

  while (lidarReadFrame(f)) {
    float start_deg = f.start_angle * 0.01f;
    float end_deg   = f.end_angle   * 0.01f;

    if (!lidar_first_frame) {
      if ((start_deg + 5.0f) < lidar_last_start_deg && lidar_frames_in_scan > 0) {
        for (int i = 0; i < SCAN_BEAMS; ++i) {
          if (!scan_valid[i]) {
            scan_ranges[i]      = INFINITY;
            scan_intensities[i] = 0.0f;
          }
        }

        stamp_now(&scan_msg.header.stamp);

        scan_msg.ranges.size      = SCAN_BEAMS;
        scan_msg.intensities.size = SCAN_BEAMS;
        RCSOFTCHECK(rcl_publish(&scan_pub, &scan_msg, NULL));

        for (int i = 0; i < SCAN_BEAMS; ++i) {
          scan_valid[i] = false;
        }
        lidar_frames_in_scan = 0;
      }
    }

    lidar_first_frame    = false;
    lidar_last_start_deg = start_deg;
    lidar_frames_in_scan++;

    if (end_deg < start_deg) end_deg += 360.0f;
    float span_deg = end_deg - start_deg;

    for (int i = 0; i < LIDAR_POINTS_PER_PACKET; ++i) {
      float frac = (LIDAR_POINTS_PER_PACKET == 1)
                 ? 0.0f
                 : (float)i / (float)(LIDAR_POINTS_PER_PACKET - 1);

      float angle_deg_ld14 = start_deg + span_deg * frac;

      while (angle_deg_ld14 >= 360.0f) angle_deg_ld14 -= 360.0f;
      while (angle_deg_ld14 < 0.0f)    angle_deg_ld14 += 360.0f;

      float angle_rad_ros = ld14DegToRosRad(angle_deg_ld14);

      float beam_f = (angle_rad_ros - ANGLE_MIN_RAD) / ANGLE_INC_RAD;
      int beam = (int)lroundf(beam_f);

      if (beam == SCAN_BEAMS) beam = 0;
      if (beam < 0 || beam >= SCAN_BEAMS) continue;

      uint16_t d_mm = f.point[i].distance;
      if (d_mm == 0) continue;

      float d_m = d_mm * 0.001f;

      if (!scan_valid[beam] || d_m < scan_ranges[beam]) {
        scan_ranges[beam]      = d_m;
        scan_intensities[beam] = (float)f.point[i].intensity;
        scan_valid[beam]       = true;
      }
    }
  }
}

// ============================= Quadrature encoder (A/B) ================================
void IRAM_ATTR onLeftA() {
  bool b = digitalRead(ENC_L_B);
  int dir = b ? -1 : +1;
  if (INVERT_LEFT_DIR) dir = -dir;
  left_ticks += dir;
}

void IRAM_ATTR onRightA() {
  bool b = digitalRead(ENC_R_B);
  int dir = b ? -1 : +1;
  if (INVERT_RIGHT_DIR) dir = -dir;
  right_ticks += dir;
}

void ticksTimerCallback(rcl_timer_t*, int64_t) {
  noInterrupts();
  wheel_ticks_buf[0] = left_ticks;
  wheel_ticks_buf[1] = right_ticks;
  interrupts();

  RCSOFTCHECK(rcl_publish(&ticks_pub, &ticks_msg, NULL));
}

// ======================================== Setup ========================================
void setup() {
  Serial.begin(115200);
  delay(500);

  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  delay(2000);

  // Motor pins
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(L_PWM, 0);
  ledcAttachPin(R_PWM, 1);

  safetyStop();

  // IMU I2C + BNO085
  Wire.begin(21, 22, 400000);
  bool imu_ok = bnoInit();
  if (!imu_ok) {
    Serial.println("BNO085 init failed");
  } else {
    Serial.println("BNO085 init OK");
  }

  // LiDAR
  Serial2.begin(230400, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  // Encoders
  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), onLeftA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), onRightA, RISING);

  // micro-ROS init
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_diffdrive", "", &support));

  // Time sync with agent
  rmw_ret_t sync_ret = rmw_uros_sync_session(1000);
  (void)sync_ret;

  // cmd_vel subscriber
  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  // IMU publisher
  RCCHECK(rclc_publisher_init_default(
      &imu_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data"));

  imu_msg.header.frame_id.data     = (char*)"imu_link";
  imu_msg.header.frame_id.size     = strlen("imu_link");
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

  RCCHECK(rclc_timer_init_default(
      &imu_timer, &support, RCL_MS_TO_NS(10), imuTimerCallback));

  // wheel_ticks publisher
  RCCHECK(rclc_publisher_init_default(
      &ticks_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "wheel_ticks"));

  ticks_msg.layout.dim.size     = 0;
  ticks_msg.layout.dim.capacity = 0;
  ticks_msg.layout.data_offset  = 0;
  ticks_msg.data.data           = wheel_ticks_buf;
  ticks_msg.data.size           = 2;
  ticks_msg.data.capacity       = 2;

  RCCHECK(rclc_timer_init_default(
      &ticks_timer, &support, RCL_MS_TO_NS(50), ticksTimerCallback));

  // LiDAR /scan publisher
  RCCHECK(rclc_publisher_init_default(
      &scan_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
      "scan"));

  const char *lidar_frame = "base_laser";
  scan_msg.header.frame_id.data     = (char*)lidar_frame;
  scan_msg.header.frame_id.size     = strlen(lidar_frame);
  scan_msg.header.frame_id.capacity = scan_msg.header.frame_id.size + 1;

  scan_msg.angle_min       = ANGLE_MIN_RAD;
  scan_msg.angle_max       = ANGLE_MAX_RAD;
  scan_msg.angle_increment = ANGLE_INC_RAD;
  scan_msg.range_min       = 0.10f;
  scan_msg.range_max       = 8.0f;
  scan_msg.scan_time       = 0.0f;
  scan_msg.time_increment  = 0.0f;

  scan_msg.ranges.data       = scan_ranges;
  scan_msg.ranges.capacity   = SCAN_BEAMS;
  scan_msg.ranges.size       = 0;

  scan_msg.intensities.data     = scan_intensities;
  scan_msg.intensities.capacity = SCAN_BEAMS;
  scan_msg.intensities.size     = 0;

  for (int i = 0; i < SCAN_BEAMS; ++i) {
    scan_ranges[i]      = INFINITY;
    scan_intensities[i] = 0.0f;
    scan_valid[i]       = false;
  }

  // executor: 1 subscription + 2 timers = 3 handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &ticks_timer));

  Serial.println("ESP32 robot controller ready.");
}

// ============================= Loop =====================================================
void loop() {
  // Process ROS callbacks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // Update motors every loop, not only when a new ROS message arrives
  applyMotorPwms(false);

  // Safety timeout
  checkCmdTimeout();

  // LiDAR parsing / publishing
  lidarSpinOnce();

  delay(2);
}
