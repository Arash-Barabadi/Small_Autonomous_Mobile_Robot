// ============================= Standard and micro-ROS Libraries =============================
#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <math.h>  // added for fabsf()

// [MOD] Added for I2C (MPU-6050)
#include <Wire.h>
// ============================= ROS 2 Libraries =============================
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// [MOD] Added IMU message
#include <sensor_msgs/msg/imu.h>

// [ENC] NEW: publish encoder ticks as Int32MultiArray [left, right]
#include <std_msgs/msg/int32_multi_array.h>

// ============================= Wi-Fi Credentials =============================

//An IP address (e.g., 192.168.178.164) identifies a device on the network (like PC).
// A port number (e.g., 8888) identifies a specific service or application running on that device.

// ============================= micro-ROS Agent IP and Port =============================
//Git fixed: changed from `const char*` to `char[]`
char ssid[] = "FRITZ!Box 7560 IQ";             // Replace with your Wi-Fi SSID
char password[] = "28715329684835834345";      // Replace with your Wi-Fi password
IPAddress agent_ip(192, 168, 178, 164);         // Your PC IP
const uint16_t agent_port = 8888;

// ============================= Motor Driver Pin Definitions =============================
const int R_IN1 = 19 ;
const int R_IN2 = 18;
const int R_PWM = 17;

const int L_IN1 = 27;
const int L_IN2 = 26;
const int L_PWM = 14 ;

// [ENC] NEW: encoder digital outputs (MH sensor D0 pins)
const int ENC_L_D0 = 34;   // left encoder -> GPIO34 (input-only, no internal pull-up)
const int ENC_R_D0 = 35;   // right encoder -> GPIO35 (input-only, no internal pull-up)

// ---------- Added: tuning & normalization constants ----------
static constexpr float TRACK = 0.20f;     // wheel separation (m), adjust accordingly
static constexpr float MAX_V = 0.50f;     // m/s corresponding to "full-scale" forward command
static constexpr float MAX_W = 2.00f;     // rad/s corresponding to "full-scale" yaw command

// Per-wheel gain to trim straight driving (increase the slower side slightly)
static constexpr float K_LEFT  = 1.00f;
static constexpr float K_RIGHT = 1.00f;

// Minimum duty to overcome static friction (0..255). Raise if wheels hesitate at low speeds.
static constexpr uint8_t MIN_DUTY = 20;

// Small deadband on normalized speed commands
static constexpr float DEADBAND = 0.01f;

// ============================= ROS 2 micro-ROS Entities =============================

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;

// IMU publisher + timer
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;
rcl_timer_t imu_timer;

// [ENC] NEW: encoder ticks publisher + timer
rcl_publisher_t ticks_pub;
std_msgs__msg__Int32MultiArray ticks_msg;
rcl_timer_t ticks_timer;
// [ENC] NEW: static storage for the Int32MultiArray data (size=2)
static int32_t wheel_ticks_buf[2] = {0, 0};

// ============================= Error Checking Macros =============================
#define RCCHECK(fn) { if ((fn) != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t _rc = (fn); (void)_rc; }

// ============================= Error Loop Function =============================
void error_loop() {
  while (true) delay(100);
}

// ---------- Added: helper to map normalized speed [-1,1] to signed PWM with deadband & MIN_DUTY ----------
static inline int16_t speedToPwm(float s_norm) {
  if (s_norm >  1.0f) s_norm =  1.0f;
  if (s_norm < -1.0f) s_norm = -1.0f;
  if (fabsf(s_norm) < DEADBAND) return 0;

  float mag = fabsf(s_norm);
  uint8_t duty = (uint8_t)(MIN_DUTY + mag * (255 - MIN_DUTY));
  return (s_norm >= 0.0f) ? (int16_t)duty : -(int16_t)duty;
}

// ============================= Set Motor Speed and Direction =============================
void setMotor(int in1, int in2, int pwm_chan, int16_t speed_pwm) {
  uint8_t duty = (uint8_t)constrain((int)abs(speed_pwm), 0, 255);

  if (speed_pwm > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (speed_pwm < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  } else {
    // ACTIVE BRAKE for fast stop
    digitalWrite(in1, HIGH); digitalWrite(in2, HIGH);
  }
  ledcWrite(pwm_chan, duty);
}

// ============================= Callback for "cmd_vel" Subscription =============================
void cmdVelCallback(const void *msgin) {
  const auto *msg = (const geometry_msgs__msg__Twist *)msgin;

  float v = msg->linear.x  / MAX_V;
  float w = msg->angular.z / MAX_W;

  if (v >  1.0f) v =  1.0f; if (v < -1.0f) v = -1.0f;
  if (w >  1.0f) w =  1.0f; if (w < -1.0f) w = -1.0f;

  float left_norm  = (v - w * TRACK / 2.0f);
  float right_norm = (v + w * TRACK / 2.0f);

  left_norm  *= K_LEFT;
  right_norm *= K_RIGHT;

  int16_t left_pwm  = speedToPwm(left_norm);
  int16_t right_pwm = speedToPwm(right_norm);

  setMotor(L_IN1, L_IN2, 0, left_pwm);
  setMotor(R_IN1, R_IN2, 1, right_pwm);
}

// ============================= [MOD] MPU-6050: minimal driver =============================
// Address (AD0=GND→0x68, AD0=3V3→0x69)
static const uint8_t MPU_ADDR       = 0x68;
static const uint8_t REG_SMPLRT_DIV = 0x19;
static const uint8_t REG_CONFIG     = 0x1A;
static const uint8_t REG_GYRO_CFG   = 0x1B;
static const uint8_t REG_ACCEL_CFG  = 0x1C;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_ACCEL_XOUT = 0x3B;

inline void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}
inline void mpuReadBytes(uint8_t start, uint8_t *buf, size_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(start);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, (int)len);
  for (size_t i = 0; i < len && Wire.available(); ++i) buf[i] = Wire.read();
}

bool mpuInit() {
  mpuWrite(REG_PWR_MGMT_1, 0x01); // wake, PLL x-gyro
  delay(10);
  mpuWrite(REG_CONFIG, 0x03);     // DLPF=3 (~44Hz)
  mpuWrite(REG_GYRO_CFG, 0x00);   // ±250 dps
  mpuWrite(REG_ACCEL_CFG, 0x00);  // ±2g
  mpuWrite(REG_SMPLRT_DIV, 9);    // 1kHz/(1+9)=100Hz
  delay(50);
  return true;
}

// Biases (computed at startup)
static float ax_b=0, ay_b=0, az_b=0;
static float gx_b=0, gy_b=0, gz_b=0;

void mpuCalibrate(int samples=500) {
  long ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  uint8_t buf[14];
  for (int i=0; i<samples; ++i) {
    mpuReadBytes(REG_ACCEL_XOUT, buf, 14);
    int16_t axr = (buf[0]<<8) | buf[1];
    int16_t ayr = (buf[2]<<8) | buf[3];
    int16_t azr = (buf[4]<<8) | buf[5];
    int16_t gxr = (buf[8]<<8) | buf[9];
    int16_t gyr = (buf[10]<<8) | buf[11];
    int16_t gzr = (buf[12]<<8) | buf[13];
    ax += axr; ay += ayr; az += azr;
    gx += gxr; gy += gyr; gz += gzr;
    delay(3);
  }
  const float ACCEL_LSB_G = 16384.0f;
  const float GYRO_LSB_DPS = 131.0f;
  ax_b = (float)ax / samples / ACCEL_LSB_G;
  ay_b = (float)ay / samples / ACCEL_LSB_G;
  az_b = (float)az / samples / ACCEL_LSB_G - 1.0f;  // remove gravity
  gx_b = (float)gx / samples / GYRO_LSB_DPS;
  gy_b = (float)gy / samples / GYRO_LSB_DPS;
  gz_b = (float)gz / samples / GYRO_LSB_DPS;
}

struct ImuSample { float ax, ay, az; float gx, gy, gz; };

ImuSample mpuRead() {
  uint8_t b[14];
  mpuReadBytes(REG_ACCEL_XOUT, b, 14);
  int16_t axr = (b[0]<<8) | b[1];
  int16_t ayr = (b[2]<<8) | b[3];
  int16_t azr = (b[4]<<8) | b[5];
  int16_t gxr = (b[8]<<8) | b[9];
  int16_t gyr = (b[10]<<8) | b[11];
  int16_t gzr = (b[12]<<8) | b[13];

  const float ACCEL_LSB_G = 16384.0f;   // ±2g
  const float GYRO_LSB_DPS = 131.0f;    // ±250 dps
  const float G = 9.80665f;
  const float DEG2RAD = 0.01745329252f;

  float axg = (axr / ACCEL_LSB_G) - ax_b;
  float ayg = (ayr / ACCEL_LSB_G) - ay_b;
  float azg = (azr / ACCEL_LSB_G) - az_b;

  float gxdps = (gxr / GYRO_LSB_DPS) - gx_b;
  float gydps = (gyr / GYRO_LSB_DPS) - gy_b;
  float gzdps = (gzr / GYRO_LSB_DPS) - gz_b;

  ImuSample s;
  s.ax = axg * G;  s.ay = ayg * G;  s.az = azg * G;
  s.gx = gxdps * DEG2RAD; s.gy = gydps * DEG2RAD; s.gz = gzdps * DEG2RAD;
  return s;
}

// Simple complementary filter state
static float roll=0, pitch=0, yaw=0;
static uint32_t prev_us = 0;

// Euler → quaternion
void eulerToQuat(float roll, float pitch, float yaw, float &qx, float &qy, float &qz, float &qw) {
  float cr = cosf(roll*0.5f), sr = sinf(roll*0.5f);
  float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
  float cy = cosf(yaw*0.5f), sy = sinf(yaw*0.5f);
  qw = cr*cp*cy + sr*sp*sy;
  qx = sr*cp*cy - cr*sp*sy;
  qy = cr*sp*cy + sr*cp*sy;
  qz = cr*cp*sy - sr*sp*cy;
}

// IMU publish timer @100 Hz
void imuTimerCallback(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/) {
  uint32_t now = micros();
  float dt = prev_us ? (now - prev_us) * 1e-6f : 0.01f;
  prev_us = now;

  ImuSample s = mpuRead();

  float roll_acc  = atan2f(s.ay, s.az);
  float pitch_acc = atan2f(-s.ax, sqrtf(s.ay*s.ay + s.az*s.az));

  const float alpha = 0.98f;
  roll  = alpha * (roll  + s.gx * dt) + (1.0f - alpha) * roll_acc;
  pitch = alpha * (pitch + s.gy * dt) + (1.0f - alpha) * pitch_acc;
  yaw   = yaw + s.gz * dt;

  imu_msg.header.stamp.sec = 0;           // (no time sync yet)
  imu_msg.header.stamp.nanosec = 0;

  float qx,qy,qz,qw; eulerToQuat(roll, pitch, yaw, qx, qy, qz, qw);
  imu_msg.orientation.x = qx;
  imu_msg.orientation.y = qy;
  imu_msg.orientation.z = qz;
  imu_msg.orientation.w = qw;

  imu_msg.angular_velocity.x = s.gx;
  imu_msg.angular_velocity.y = s.gy;
  imu_msg.angular_velocity.z = s.gz;

  imu_msg.linear_acceleration.x = s.ax;
  imu_msg.linear_acceleration.y = s.ay;
  imu_msg.linear_acceleration.z = s.az;

  for (int i=0;i<9;i++) {
    imu_msg.orientation_covariance[i] = 0.0;
    imu_msg.angular_velocity_covariance[i] = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }
  imu_msg.orientation_covariance[0] = imu_msg.orientation_covariance[4] = imu_msg.orientation_covariance[8] = 0.05;
  imu_msg.angular_velocity_covariance[0] = imu_msg.angular_velocity_covariance[4] = imu_msg.angular_velocity_covariance[8] = 0.02;
  imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4] = imu_msg.linear_acceleration_covariance[8] = 0.2;

  RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
}

// [ENC] NEW: encoder tick counters and ISRs
volatile int32_t left_ticks = 0;
volatile int32_t right_ticks = 0;

void IRAM_ATTR onLeftTick()  { left_ticks++; }
void IRAM_ATTR onRightTick() { right_ticks++; }

// [ENC] NEW: publish encoder ticks @20 Hz
void ticksTimerCallback(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/) {
  // snapshot volatile counters
  wheel_ticks_buf[0] = left_ticks;   // left
  wheel_ticks_buf[1] = right_ticks;  // right
  // header not present in Int32MultiArray; publish raw array
  RCSOFTCHECK(rcl_publish(&ticks_pub, &ticks_msg, NULL));
}

// ======================================== Setup Function ========================================
void setup() {
  Serial.begin(115200);
  delay(500);

  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  delay(2000);

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

  // I²C init for MPU-6050 (SDA=21, SCL=22 @ 400k)
  Wire.begin(21, 22, 400000);
  mpuInit();
  mpuCalibrate(600);

  // [ENC] NEW: encoder GPIOs + interrupts
  // NOTE: GPIO34/35 are input-only and have NO internal pull-ups. Use INPUT and ensure valid digital signal or external pull-ups.
  pinMode(ENC_L_D0, INPUT);
  pinMode(ENC_R_D0, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_D0), onLeftTick,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_D0), onRightTick, RISING);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_diffdrive", "", &support));

  // cmd_vel subscription
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // IMU publisher
  RCCHECK(rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data"));

  imu_msg.header.frame_id.data = (char*)"imu_link";
  imu_msg.header.frame_id.size = strlen("imu_link");
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

  const unsigned int imu_period_ms = 10; // 100 Hz
  RCCHECK(rclc_timer_init_default(
    &imu_timer, &support, RCL_MS_TO_NS(imu_period_ms), imuTimerCallback));

  // [ENC] NEW: ticks publisher init on /wheel_ticks
  RCCHECK(rclc_publisher_init_default(
    &ticks_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "wheel_ticks"));

  // [ENC] NEW: initialize Int32MultiArray data buffer (size=2)
  ticks_msg.layout.dim.size = 0;       // no special layout
  ticks_msg.layout.dim.capacity = 0;
  ticks_msg.layout.data_offset = 0;
  ticks_msg.data.data = wheel_ticks_buf;
  ticks_msg.data.size = 2;
  ticks_msg.data.capacity = 2;

  // [ENC] NEW: ticks timer at 20 Hz (50 ms)
  const unsigned int ticks_period_ms = 50; // 20 Hz
  RCCHECK(rclc_timer_init_default(
    &ticks_timer, &support, RCL_MS_TO_NS(ticks_period_ms), ticksTimerCallback));

  // [ENC] CHANGED: executor now handles 1 sub + 2 timers (size: 3)
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  // [ENC] NEW:
  RCCHECK(rclc_executor_add_timer(&executor, &ticks_timer));
}

// ============================= Loop Function =============================
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}
