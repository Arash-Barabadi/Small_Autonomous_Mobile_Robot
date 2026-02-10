// ============================= Standard and micro-ROS Libraries =============================
#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <math.h>
#include <Wire.h>

// micro-ROS time API
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
static constexpr uint32_t CMD_TIMEOUT_US = 250000; // 250 ms

// ============================= Encoder Pin Definitions =============================
const int ENC_L_A = 34;
const int ENC_L_B = 32;
const int ENC_R_A = 35;
const int ENC_R_B = 33;

static constexpr bool INVERT_LEFT_DIR  = false;
static constexpr bool INVERT_RIGHT_DIR = false;

// ---------- Tuning & normalization ----------
static constexpr float TRACK     = 0.20f;  // wheel separation (m)
static constexpr float MAX_V     = 0.50f;  // m/s at full-scale cmd
static constexpr float MAX_W     = 2.00f;  // rad/s at full-scale cmd
static constexpr float K_LEFT    = 1.00f;
static constexpr float K_RIGHT   = 1.00f;
static constexpr uint8_t MIN_DUTY = 20;
static constexpr float DEADBAND  = 0.01f;

// ============================= LiDAR (LD14P) =============================================
static constexpr int LIDAR_RX_PIN = 16;
static constexpr int LIDAR_TX_PIN = 4;

static constexpr uint8_t LIDAR_HEADER            = 0x54;
static constexpr int     LIDAR_POINTS_PER_PACKET = 12;

static constexpr int   SCAN_BEAMS    = 720;
static constexpr float ANGLE_MIN_RAD = 0.0f;
static constexpr float ANGLE_INC_RAD = (2.0f * 3.14159265f) / SCAN_BEAMS;
static constexpr float ANGLE_MAX_RAD = ANGLE_MIN_RAD + ANGLE_INC_RAD * (SCAN_BEAMS - 1);

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

// ============================= Helper: stamp with ROS time ===============================
static inline void stamp_now(builtin_interfaces__msg__Time *t)
{
  // time in nanoseconds from micro-ROS agent (sync via rmw_uros_sync_session)
  int64_t now_ns = rmw_uros_epoch_nanos();

  if (now_ns <= 0) {
    t->sec     = 0;
    t->nanosec = 0;
  } else {
    t->sec     = (int32_t)(now_ns / 1000000000LL);
    t->nanosec = (uint32_t)(now_ns % 1000000000LL);
  }
}

// ================================= Motor drive ==========================================
void setMotor(int in1, int in2, int pwm_chan, int16_t speed_pwm) {
  uint8_t duty = (uint8_t)constrain((int)abs(speed_pwm), 0, 255);

  if (speed_pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed_pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);  // active brake
  }

  ledcWrite(pwm_chan, duty);
}

static inline void safetyStop() {
  setMotor(L_IN1, L_IN2, 0, 0);
  setMotor(R_IN1, R_IN2, 1, 0);
}

void error_loop() {
  while (true) delay(100);
}

static inline int16_t speedToPwm(float s_norm) {
  if (s_norm >  1.0f) s_norm =  1.0f;
  if (s_norm < -1.0f) s_norm = -1.0f;
  if (fabsf(s_norm) < DEADBAND) return 0;
  float mag = fabsf(s_norm);
  uint8_t duty = (uint8_t)(MIN_DUTY + mag * (255 - MIN_DUTY));
  return (s_norm >= 0.0f) ? (int16_t)duty : -(int16_t)duty;
}

// ================================= cmd_vel callback =====================================
void cmdVelCallback(const void *msgin) {
  last_cmd_us = micros();

  const auto *msg = (const geometry_msgs__msg__Twist *)msgin;

  float v = constrain(msg->linear.x  / MAX_V, -1.0f, 1.0f);
  float w = constrain(msg->angular.z / MAX_W, -1.0f, 1.0f);

  float left_norm  = (v - w * TRACK / 2.0f) * K_LEFT;
  float right_norm = (v + w * TRACK / 2.0f) * K_RIGHT;

  setMotor(L_IN1, L_IN2, 0, speedToPwm(left_norm));
  setMotor(R_IN1, R_IN2, 1, speedToPwm(right_norm));
}

void checkCmdTimeout() {
  uint32_t now = micros();
  if (last_cmd_us && (now - last_cmd_us > CMD_TIMEOUT_US)) {
    safetyStop();
  }
}

// ============================= MPU-6050 minimal driver ==================================
static const uint8_t MPU_ADDR       = 0x68;
static const uint8_t REG_SMPLRT_DIV = 0x19;
static const uint8_t REG_CONFIG     = 0x1A;
static const uint8_t REG_GYRO_CFG   = 0x1B;
static const uint8_t REG_ACCEL_CFG  = 0x1C;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_ACCEL_XOUT = 0x3B;

inline void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
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
  mpuWrite(REG_PWR_MGMT_1, 0x01); delay(10);
  mpuWrite(REG_CONFIG,     0x03);
  mpuWrite(REG_GYRO_CFG,   0x00);
  mpuWrite(REG_ACCEL_CFG,  0x00);
  mpuWrite(REG_SMPLRT_DIV, 9);
  delay(50);
  return true;
}

static float ax_b=0, ay_b=0, az_b=0, gx_b=0, gy_b=0, gz_b=0;

void mpuCalibrate(int samples = 500) {
  long ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
  uint8_t b[14];
  for (int i=0; i<samples; ++i) {
    mpuReadBytes(REG_ACCEL_XOUT,b,14);
    int16_t axr=(b[0]<<8)|b[1], ayr=(b[2]<<8)|b[3], azr=(b[4]<<8)|b[5];
    int16_t gxr=(b[8]<<8)|b[9], gyr=(b[10]<<8)|b[11], gzr=(b[12]<<8)|b[13];
    ax+=axr; ay+=ayr; az+=azr; gx+=gxr; gy+=gyr; gz+=gzr; delay(3);
  }
  const float ACCEL_LSB_G=16384.0f, GYRO_LSB_DPS=131.0f;
  ax_b=(float)ax/samples/ACCEL_LSB_G;
  ay_b=(float)ay/samples/ACCEL_LSB_G;
  az_b=(float)az/samples/ACCEL_LSB_G-1.0f;
  gx_b=(float)gx/samples/GYRO_LSB_DPS;
  gy_b=(float)gy/samples/GYRO_LSB_DPS;
  gz_b=(float)gz/samples/GYRO_LSB_DPS;
}

struct ImuSample { float ax,ay,az,gx,gy,gz; };

ImuSample mpuRead() {
  uint8_t b[14];
  mpuReadBytes(REG_ACCEL_XOUT,b,14);
  int16_t axr=(b[0]<<8)|b[1], ayr=(b[2]<<8)|b[3], azr=(b[4]<<8)|b[5];
  int16_t gxr=(b[8]<<8)|b[9], gyr=(b[10]<<8)|b[11], gzr=(b[12]<<8)|b[13];
  const float ACCEL_LSB_G=16384.0f, GYRO_LSB_DPS=131.0f, G=9.80665f, DEG2RAD=0.01745329252f;
  ImuSample s;
  s.ax=((axr/ACCEL_LSB_G)-ax_b)*G;
  s.ay=((ayr/ACCEL_LSB_G)-ay_b)*G;
  s.az=((azr/ACCEL_LSB_G)-az_b)*G;
  s.gx=((gxr/GYRO_LSB_DPS)-gx_b)*DEG2RAD;
  s.gy=((gyr/GYRO_LSB_DPS)-gy_b)*DEG2RAD;
  s.gz=((gzr/GYRO_LSB_DPS)-gz_b)*DEG2RAD;
  return s;
}

static float roll=0, pitch=0, yaw=0;
static uint32_t prev_us=0;

void eulerToQuat(float r,float p,float y,float &qx,float &qy,float &qz,float &qw) {
  float cr=cosf(r*0.5f), sr=sinf(r*0.5f);
  float cp=cosf(p*0.5f), sp=sinf(p*0.5f);
  float cy=cosf(y*0.5f), sy=sinf(y*0.5f);
  qw=cr*cp*cy + sr*sp*sy;
  qx=sr*cp*cy - cr*sp*sy;
  qy=cr*sp*cy + sr*cp*sy;
  qz=cr*cp*sy - sr*sp*cy;
}

void imuTimerCallback(rcl_timer_t*, int64_t) {
  uint32_t now=micros();
  float dt = prev_us ? (now-prev_us)*1e-6f : 0.01f;
  prev_us=now;

  ImuSample s=mpuRead();
  float roll_acc  = atan2f(s.ay,s.az);
  float pitch_acc = atan2f(-s.ax, sqrtf(s.ay*s.ay+s.az*s.az));
  const float alpha=0.98f;
  roll  = alpha*(roll  + s.gx*dt) + (1.0f-alpha)*roll_acc;
  pitch = alpha*(pitch + s.gy*dt) + (1.0f-alpha)*pitch_acc;
  yaw   = yaw + s.gz*dt;

  // timestamp IMU message
  stamp_now(&imu_msg.header.stamp);

  float qx,qy,qz,qw;
  eulerToQuat(roll,pitch,yaw,qx,qy,qz,qw);
  imu_msg.orientation.x=qx;
  imu_msg.orientation.y=qy;
  imu_msg.orientation.z=qz;
  imu_msg.orientation.w=qw;
  imu_msg.angular_velocity.x=s.gx;
  imu_msg.angular_velocity.y=s.gy;
  imu_msg.angular_velocity.z=s.gz;
  imu_msg.linear_acceleration.x=s.ax;
  imu_msg.linear_acceleration.y=s.ay;
  imu_msg.linear_acceleration.z=s.az;

  for(int i=0;i<9;i++){
    imu_msg.orientation_covariance[i]=0.0;
    imu_msg.angular_velocity_covariance[i]=0.0;
    imu_msg.linear_acceleration_covariance[i]=0.0;
  }
  imu_msg.orientation_covariance[0]=imu_msg.orientation_covariance[4]=imu_msg.orientation_covariance[8]=0.05;
  imu_msg.angular_velocity_covariance[0]=imu_msg.angular_velocity_covariance[4]=imu_msg.angular_velocity_covariance[8]=0.02;
  imu_msg.linear_acceleration_covariance[0]=imu_msg.linear_acceleration_covariance[4]=imu_msg.linear_acceleration_covariance[8]=0.2;

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
  const float DEG2RAD = 0.01745329252f;

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

        // timestamp + sizes before publishing
        stamp_now(&scan_msg.header.stamp);
        //scan_msg.header.stamp.sec = 0;
        //scan_msg.header.stamp.nanosec = 0;


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
      float frac = (LIDAR_POINTS_PER_PACKET == 1) ? 0.0f : (float)i / (float)(LIDAR_POINTS_PER_PACKET - 1);
      float angle_deg = start_deg + span_deg * frac;

      while (angle_deg >= 360.0f) angle_deg -= 360.0f;
      while (angle_deg < 0.0f)   angle_deg += 360.0f;

      float beam_f = (angle_deg * DEG2RAD - ANGLE_MIN_RAD) / ANGLE_INC_RAD;
      int   beam   = (int)lroundf(beam_f);
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
volatile int32_t left_ticks  = 0;
volatile int32_t right_ticks = 0;

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
  wheel_ticks_buf[0] = left_ticks;
  wheel_ticks_buf[1] = right_ticks;
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

  // IMU I2C
  Wire.begin(21, 22, 400000);
  mpuInit();
  mpuCalibrate(600);

  // LiDAR
  Serial2.begin(230400, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  // Encoders
  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), onLeftA,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), onRightA, RISING);

  // micro-ROS init
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_diffdrive", "", &support));


    // Now that micro-ROS session is running, sync time with the Agent
  rmw_ret_t sync_ret = rmw_uros_sync_session(1000);  // 1 s timeout
  if (sync_ret != RMW_RET_OK) {
    // optional: Serial.println("Time sync failed");
  }


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

  const unsigned int imu_period_ms = 10;
  RCCHECK(rclc_timer_init_default(
      &imu_timer, &support, RCL_MS_TO_NS(imu_period_ms), imuTimerCallback));

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

  const unsigned int ticks_period_ms = 50;
  RCCHECK(rclc_timer_init_default(
      &ticks_timer, &support, RCL_MS_TO_NS(ticks_period_ms), ticksTimerCallback));

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

  // executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &ticks_timer));
}

// ============================= Loop =====================================================
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
  checkCmdTimeout();
  lidarSpinOnce();
  delay(5);
}
