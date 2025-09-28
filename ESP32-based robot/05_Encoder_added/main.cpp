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

// [ENC] publish encoder ticks as Int32MultiArray [left, right]
#include <std_msgs/msg/int32_multi_array.h>

// ============================= micro-ROS Agent IP and Port =============================
char ssid[] = "FRITZ!Box 7560 IQ";
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

// ============================= Encoder Pin Definitions (A/B per wheel) =====================
// Use A on input-only pins (34/35), B on GPIOs with pullups (32/33) if needed.
const int ENC_L_A = 34;   // Left encoder A (input only, no internal pull-up)
const int ENC_L_B = 32;   // Left encoder B (supports INPUT_PULLUP)
const int ENC_R_A = 35;   // Right encoder A (input only, no internal pull-up)
const int ENC_R_B = 33;   // Right encoder B (supports INPUT_PULLUP)

// If your A/B wiring gives reversed direction, flip these booleans.
static constexpr bool INVERT_LEFT_DIR  = false;
static constexpr bool INVERT_RIGHT_DIR = false;

// ---------- Tuning & normalization ----------
static constexpr float TRACK = 0.20f;     // wheel separation (m)
static constexpr float MAX_V = 0.50f;     // m/s at full-scale cmd
static constexpr float MAX_W = 2.00f;     // rad/s at full-scale cmd
static constexpr float K_LEFT  = 1.00f;
static constexpr float K_RIGHT = 1.00f;
static constexpr uint8_t MIN_DUTY = 20;
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

// Encoder ticks publisher + timer
rcl_publisher_t ticks_pub;
std_msgs__msg__Int32MultiArray ticks_msg;
rcl_timer_t ticks_timer;
static int32_t wheel_ticks_buf[2] = {0, 0};

// ============================= Error Checking Macros =============================
#define RCCHECK(fn) { if ((fn) != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t _rc = (fn); (void)_rc; }

// ============================= Error Loop Function =============================
void error_loop() { while (true) delay(100); }

// ---------- map normalized speed [-1,1] to signed PWM ----------
static inline int16_t speedToPwm(float s_norm) {
  if (s_norm >  1.0f) s_norm =  1.0f;
  if (s_norm < -1.0f) s_norm = -1.0f;
  if (fabsf(s_norm) < DEADBAND) return 0;
  float mag = fabsf(s_norm);
  uint8_t duty = (uint8_t)(MIN_DUTY + mag * (255 - MIN_DUTY));
  return (s_norm >= 0.0f) ? (int16_t)duty : -(int16_t)duty;
}

// ============================= Motor drive =============================
void setMotor(int in1, int in2, int pwm_chan, int16_t speed_pwm) {
  uint8_t duty = (uint8_t)constrain((int)abs(speed_pwm), 0, 255);
  if (speed_pwm > 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (speed_pwm < 0) { digitalWrite(in1, LOW); digitalWrite(in2, HIGH); }
  else { digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); } // active brake
  ledcWrite(pwm_chan, duty);
}

// ============================= cmd_vel callback =============================
void cmdVelCallback(const void *msgin) {
  const auto *msg = (const geometry_msgs__msg__Twist *)msgin;

  float v = msg->linear.x  / MAX_V;
  float w = msg->angular.z / MAX_W;

  v = constrain(v, -1.0f, 1.0f);
  w = constrain(w, -1.0f, 1.0f);

  float left_norm  = (v - w * TRACK / 2.0f) * K_LEFT;
  float right_norm = (v + w * TRACK / 2.0f) * K_RIGHT;

  int16_t left_pwm  = speedToPwm(left_norm);
  int16_t right_pwm = speedToPwm(right_norm);

  setMotor(L_IN1, L_IN2, 0, left_pwm);
  setMotor(R_IN1, R_IN2, 1, right_pwm);
}

// ============================= MPU-6050 minimal driver =============================
static const uint8_t MPU_ADDR       = 0x68;
static const uint8_t REG_SMPLRT_DIV = 0x19;
static const uint8_t REG_CONFIG     = 0x1A;
static const uint8_t REG_GYRO_CFG   = 0x1B;
static const uint8_t REG_ACCEL_CFG  = 0x1C;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_ACCEL_XOUT = 0x3B;

inline void mpuWrite(uint8_t reg, uint8_t val){ Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission(); }
inline void mpuReadBytes(uint8_t start, uint8_t *buf, size_t len){
  Wire.beginTransmission(MPU_ADDR); Wire.write(start); Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, (int)len); for (size_t i=0;i<len && Wire.available();++i) buf[i]=Wire.read(); }

bool mpuInit(){
  mpuWrite(REG_PWR_MGMT_1, 0x01); delay(10);
  mpuWrite(REG_CONFIG, 0x03);
  mpuWrite(REG_GYRO_CFG, 0x00);
  mpuWrite(REG_ACCEL_CFG, 0x00);
  mpuWrite(REG_SMPLRT_DIV, 9);
  delay(50); return true;
}

static float ax_b=0, ay_b=0, az_b=0, gx_b=0, gy_b=0, gz_b=0;

void mpuCalibrate(int samples=500){
  long ax=0,ay=0,az=0,gx=0,gy=0,gz=0; uint8_t b[14];
  for(int i=0;i<samples;++i){
    mpuReadBytes(REG_ACCEL_XOUT,b,14);
    int16_t axr=(b[0]<<8)|b[1], ayr=(b[2]<<8)|b[3], azr=(b[4]<<8)|b[5];
    int16_t gxr=(b[8]<<8)|b[9], gyr=(b[10]<<8)|b[11], gzr=(b[12]<<8)|b[13];
    ax+=axr; ay+=ayr; az+=azr; gx+=gxr; gy+=gyr; gz+=gzr; delay(3);
  }
  const float ACCEL_LSB_G=16384.0f, GYRO_LSB_DPS=131.0f;
  ax_b=(float)ax/samples/ACCEL_LSB_G; ay_b=(float)ay/samples/ACCEL_LSB_G; az_b=(float)az/samples/ACCEL_LSB_G-1.0f;
  gx_b=(float)gx/samples/GYRO_LSB_DPS; gy_b=(float)gy/samples/GYRO_LSB_DPS; gz_b=(float)gz/samples/GYRO_LSB_DPS;
}

struct ImuSample{ float ax,ay,az,gx,gy,gz; };

ImuSample mpuRead(){
  uint8_t b[14]; mpuReadBytes(REG_ACCEL_XOUT,b,14);
  int16_t axr=(b[0]<<8)|b[1], ayr=(b[2]<<8)|b[3], azr=(b[4]<<8)|b[5];
  int16_t gxr=(b[8]<<8)|b[9], gyr=(b[10]<<8)|b[11], gzr=(b[12]<<8)|b[13];
  const float ACCEL_LSB_G=16384.0f, GYRO_LSB_DPS=131.0f, G=9.80665f, DEG2RAD=0.01745329252f;
  ImuSample s;
  s.ax=((axr/ACCEL_LSB_G)-ax_b)*G; s.ay=((ayr/ACCEL_LSB_G)-ay_b)*G; s.az=((azr/ACCEL_LSB_G)-az_b)*G;
  s.gx=((gxr/GYRO_LSB_DPS)-gx_b)*DEG2RAD; s.gy=((gyr/GYRO_LSB_DPS)-gy_b)*DEG2RAD; s.gz=((gzr/GYRO_LSB_DPS)-gz_b)*DEG2RAD;
  return s;
}

static float roll=0, pitch=0, yaw=0; static uint32_t prev_us=0;

void eulerToQuat(float r,float p,float y,float &qx,float &qy,float &qz,float &qw){
  float cr=cosf(r*0.5f), sr=sinf(r*0.5f), cp=cosf(p*0.5f), sp=sinf(p*0.5f), cy=cosf(y*0.5f), sy=sinf(y*0.5f);
  qw=cr*cp*cy + sr*sp*sy; qx=sr*cp*cy - cr*sp*sy; qy=cr*sp*cy + sr*cp*sy; qz=cr*cp*sy - sr*sp*cy;
}

void imuTimerCallback(rcl_timer_t*, int64_t){
  uint32_t now=micros(); float dt=prev_us? (now-prev_us)*1e-6f : 0.01f; prev_us=now;
  ImuSample s=mpuRead();
  float roll_acc=atan2f(s.ay,s.az), pitch_acc=atan2f(-s.ax, sqrtf(s.ay*s.ay+s.az*s.az));
  const float alpha=0.98f;
  roll  = alpha*(roll  + s.gx*dt) + (1.0f-alpha)*roll_acc;
  pitch = alpha*(pitch + s.gy*dt) + (1.0f-alpha)*pitch_acc;
  yaw   = yaw + s.gz*dt;

  imu_msg.header.stamp.sec = 0; imu_msg.header.stamp.nanosec = 0;
  float qx,qy,qz,qw; eulerToQuat(roll,pitch,yaw,qx,qy,qz,qw);
  imu_msg.orientation.x=qx; imu_msg.orientation.y=qy; imu_msg.orientation.z=qz; imu_msg.orientation.w=qw;
  imu_msg.angular_velocity.x=s.gx; imu_msg.angular_velocity.y=s.gy; imu_msg.angular_velocity.z=s.gz;
  imu_msg.linear_acceleration.x=s.ax; imu_msg.linear_acceleration.y=s.ay; imu_msg.linear_acceleration.z=s.az;

  for(int i=0;i<9;i++){ imu_msg.orientation_covariance[i]=0.0; imu_msg.angular_velocity_covariance[i]=0.0; imu_msg.linear_acceleration_covariance[i]=0.0; }
  imu_msg.orientation_covariance[0]=imu_msg.orientation_covariance[4]=imu_msg.orientation_covariance[8]=0.05;
  imu_msg.angular_velocity_covariance[0]=imu_msg.angular_velocity_covariance[4]=imu_msg.angular_velocity_covariance[8]=0.02;
  imu_msg.linear_acceleration_covariance[0]=imu_msg.linear_acceleration_covariance[4]=imu_msg.linear_acceleration_covariance[8]=0.2;

  RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
}

// ============================= Quadrature encoder (A/B) =============================
volatile int32_t left_ticks  = 0;
volatile int32_t right_ticks = 0;

// x1 decoding: on A rising edge, read B → decide sign
void IRAM_ATTR onLeftA() {
  bool b = digitalRead(ENC_L_B);
  int dir = b ? -1 : +1;             // A leads B => +1
  if (INVERT_LEFT_DIR) dir = -dir;
  left_ticks += dir;
}
void IRAM_ATTR onRightA() {
  bool b = digitalRead(ENC_R_B);
  int dir = b ? -1 : +1;
  if (INVERT_RIGHT_DIR) dir = -dir;
  right_ticks += dir;
}

// (Optional) keep B pins for future x4 decoding / debouncing via PCNT
// We don't attach ISRs to B here to keep ISR load low.

// publish encoder ticks @20 Hz
void ticksTimerCallback(rcl_timer_t*, int64_t){
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

  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(L_PWM, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT); pinMode(R_PWM, OUTPUT);

  ledcSetup(0, 5000, 8); ledcSetup(1, 5000, 8);
  ledcAttachPin(L_PWM, 0); ledcAttachPin(R_PWM, 1);

  // I2C for MPU6050 (SDA=21, SCL=22)
  Wire.begin(21, 22, 400000);
  mpuInit(); mpuCalibrate(600);

  // ---- Encoder GPIOs ----
  // If your module has internal pull-ups, INPUT is fine for all.
  // If not, use INPUT for 34/35 (no PUs) and INPUT_PULLUP for 32/33.
  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_L_B, INPUT_PULLUP);   // safe even if external PUs exist
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), onLeftA,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), onRightA, RISING);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_diffdrive", "", &support));

  // cmd_vel subscription
  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  // IMU publisher
  RCCHECK(rclc_publisher_init_default(
      &imu_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data"));

  imu_msg.header.frame_id.data = (char*)"imu_link";
  imu_msg.header.frame_id.size = strlen("imu_link");
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

  const unsigned int imu_period_ms = 10; // 100 Hz
  RCCHECK(rclc_timer_init_default(
      &imu_timer, &support, RCL_MS_TO_NS(imu_period_ms), imuTimerCallback));

  // ticks publisher on /wheel_ticks
  RCCHECK(rclc_publisher_init_default(
      &ticks_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "wheel_ticks"));

  ticks_msg.layout.dim.size = 0;
  ticks_msg.layout.dim.capacity = 0;
  ticks_msg.layout.data_offset = 0;
  ticks_msg.data.data = wheel_ticks_buf;
  ticks_msg.data.size = 2;
  ticks_msg.data.capacity = 2;

  const unsigned int ticks_period_ms = 50; // 20 Hz
  RCCHECK(rclc_timer_init_default(
      &ticks_timer, &support, RCL_MS_TO_NS(ticks_period_ms), ticksTimerCallback));

  // executor: 1 sub + 2 timers
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &ticks_timer));
}

// ============================= Loop =============================
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}
