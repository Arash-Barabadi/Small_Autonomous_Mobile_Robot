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
//We're preparing our robot to receive commands from ROS 2. To do that, we need a few "tools" from micro-ROS to: 

// What it is: your mailbox. 	You say: “Please deliver messages from topic cmd_vel to this mailbox.” 
//It doesn’t hold the message itself—it’s just the channel for subscription.
//Think of it as: "Tell micro-ROS what to listen to"
rcl_subscription_t cmd_vel_sub;

// What it is: This is the actual message content (e.g., linear and angular velocity).
// Think of it as: "Stores the contents of the message received"
geometry_msgs__msg__Twist cmd_vel_msg;

// What it is: A helper that checks for new messages and calls the right function when one arrives.
// Think of it as: "The smart assistant that listens and reacts when new commands come in."
rclc_executor_t executor;

// What it is: A setup tool that initializes the whole ROS 2 communication system on your ESP32.
// Think of it as: "The person who sets up the communication environment for you."
rclc_support_t support;


// What it is: Your robot’s identity in the ROS 2 world.
// In this case: The node might be named "esp32_diffdrive".
// Think of it as: "This is me, the robot, talking in the ROS 2 network."
rcl_node_t node;

//What it is: A tool that handles memory allocation in micro-ROS (required for internal structures).
//Think of it as: "The memory manager that decides where things get stored."
rcl_allocator_t allocator;

// IMU publisher + timer
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;
rcl_timer_t imu_timer;

// ============================= Error Checking Macros =============================
// This macro checks if a function (fn) was successful, and if not, it enters an infinite error loop.
#define RCCHECK(fn) { if ((fn) != RCL_RET_OK) error_loop(); }

// This macro calls a function and stores the result, but it doesn’t check for errors or take action. It's used for non-critical operations.
#define RCSOFTCHECK(fn) { rcl_ret_t _rc = (fn); (void)_rc; }

// ============================= Error Loop Function =============================
void error_loop() {
  while (true) delay(100);
}

// ---------- Added: helper to map normalized speed [-1,1] to signed PWM with deadband & MIN_DUTY ----------
static inline int16_t speedToPwm(float s_norm) {
  // Clamp to [-1,1] and apply small deadband
  if (s_norm >  1.0f) s_norm =  1.0f;
  if (s_norm < -1.0f) s_norm = -1.0f;
  if (fabsf(s_norm) < DEADBAND) return 0;

  // Scale magnitude to [MIN_DUTY..255]
  float mag = fabsf(s_norm);
  uint8_t duty = (uint8_t)(MIN_DUTY + mag * (255 - MIN_DUTY));
  return (s_norm >= 0.0f) ? (int16_t)duty : -(int16_t)duty;
}

// ============================= Set Motor Speed and Direction =============================

// Function to set motor direction and speed using two digital pins and one PWM channel
// ---------- Note: updated to take signed PWM in range [-255..255] ----------
void setMotor(int in1, int in2, int pwm_chan, int16_t speed_pwm) {

  // Calculate the PWM duty cycle: 
  // Take the absolute value of speed (so negative speed doesn’t break it),
  // Clamp it between 0 and 255 to stay within 8-bit limits,
  // Convert the result to an unsigned 8-bit integer (0–255)
  uint8_t duty = (uint8_t)constrain((int)abs(speed_pwm), 0, 255);
  
  // Set motor direction:

  // If speed is positive, motor moves forward (IN1 = HIGH, IN2 = LOW)
  if (speed_pwm > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  
  // If speed is negative, motor moves in reverse (IN1 = LOW, IN2 = HIGH)
  } else if (speed_pwm < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  
  // If speed is zero, stop the motor (both IN1 and IN2 LOW = brake or coast depending on driver)
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  }

  // Write the duty cycle to the selected PWM channel,
  // This controls how fast the motor spins (0 = off, 255 = full speed)
  ledcWrite(pwm_chan, duty);
}


// ============================= Callback for "cmd_vel" Subscription =============================
// ROS 2 Twist Callback Function
// This is a callback function. It gets automatically called when our robot receives a cmd_vel message from ROS 2.
void cmdVelCallback(const void *msgin) {

  // This line takes a generic pointer (msgin) and casts it to a pointer of type const geometry_msgs__msg__Twist*, 
  // and stores it in a new pointer variable called msg.
  const auto *msg = (const geometry_msgs__msg__Twist *)msgin;
  // (const geometry_msgs__msg__Twist *)msgin:
  // This is a type cast. It tells the compiler:  “Trust me — the data pointed to by msgin is of type geometry_msgs__msg__Twist.”
  // msg: Now points to the actual message (of type Twist) that was passed in generically via msgin.

  // ---------- Normalizes teleop commands to [-1,1] using chosen maxes ----------
  float v = msg->linear.x  / MAX_V;
  float w = msg->angular.z / MAX_W;

  // clamp normalized values
  if (v >  1.0f) v =  1.0f; if (v < -1.0f) v = -1.0f;
  if (w >  1.0f) w =  1.0f; if (w < -1.0f) w = -1.0f;

  // ---------- Unicycle -> differential (still in normalized space) ----------
  // const float track = 0.2f; // wheel separation (m), adjust accordingly   // (kept for your reference)
  float left_norm  = (v - w * TRACK / 2.0f);
  float right_norm = (v + w * TRACK / 2.0f);

  // ---------- Per-wheel trim to keep straight lines straight ----------
  left_norm  *= K_LEFT;
  right_norm *= K_RIGHT;

  // ---------- Convert to signed PWM and send ----------
  int16_t left_pwm  = speedToPwm(left_norm);
  int16_t right_pwm = speedToPwm(right_norm);

  setMotor(L_IN1, L_IN2, 0, left_pwm);
  setMotor(R_IN1, R_IN2, 1, right_pwm);
}

// ============================= [MOD] MPU-6050: minimal driver =============================
// Address (AD0=GND→0x68, AD0=3V3→0x69)
static const uint8_t MPU_ADDR       = 0x68;     // [MOD]
static const uint8_t REG_SMPLRT_DIV = 0x19;     // [MOD]
static const uint8_t REG_CONFIG     = 0x1A;     // [MOD]
static const uint8_t REG_GYRO_CFG   = 0x1B;     // [MOD]
static const uint8_t REG_ACCEL_CFG  = 0x1C;     // [MOD]
static const uint8_t REG_PWR_MGMT_1 = 0x6B;     // [MOD]
static const uint8_t REG_ACCEL_XOUT = 0x3B;     // [MOD]

inline void mpuWrite(uint8_t reg, uint8_t val) {  // [MOD]
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}
inline void mpuReadBytes(uint8_t start, uint8_t *buf, size_t len) {  // [MOD]
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(start);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, (int)len);
  for (size_t i = 0; i < len && Wire.available(); ++i) buf[i] = Wire.read();
}

bool mpuInit() {                                   // [MOD]
  mpuWrite(REG_PWR_MGMT_1, 0x01); // wake, PLL x-gyro
  delay(10);
  mpuWrite(REG_CONFIG, 0x03);     // DLPF=3 (~44Hz)
  mpuWrite(REG_GYRO_CFG, 0x00);   // ±250 dps
  mpuWrite(REG_ACCEL_CFG, 0x00);  // ±2g
  mpuWrite(REG_SMPLRT_DIV, 9);    // 1kHz/(1+9)=100Hz
  delay(50);
  return true;
}

// Biases (computed at startup)  [MOD]
static float ax_b=0, ay_b=0, az_b=0; // accel in g (az includes gravity)
static float gx_b=0, gy_b=0, gz_b=0; // gyro in deg/s

void mpuCalibrate(int samples=500) {               // [MOD]
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

struct ImuSample { float ax, ay, az; float gx, gy, gz; };  // [MOD]

ImuSample mpuRead() {                          // [MOD]
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
  s.ax = axg * G;  s.ay = ayg * G;  s.az = azg * G;      // m/s^2
  s.gx = gxdps * DEG2RAD; s.gy = gydps * DEG2RAD; s.gz = gzdps * DEG2RAD; // rad/s
  return s;
}

// [MOD] Simple complementary filter state
static float roll=0, pitch=0, yaw=0;
static uint32_t prev_us = 0;

// [MOD] Euler → quaternion
void eulerToQuat(float roll, float pitch, float yaw, float &qx, float &qy, float &qz, float &qw) {
  float cr = cosf(roll*0.5f), sr = sinf(roll*0.5f);
  float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
  float cy = cosf(yaw*0.5f), sy = sinf(yaw*0.5f);
  qw = cr*cp*cy + sr*sp*sy;
  qx = sr*cp*cy - cr*sp*sy;
  qy = cr*sp*cy + sr*cp*sy;
  qz = cr*cp*sy - sr*sp*cy;
}

// [MOD] IMU publish timer @100 Hz
void imuTimerCallback(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/) {
  uint32_t now = micros();
  float dt = prev_us ? (now - prev_us) * 1e-6f : 0.01f;
  prev_us = now;

  ImuSample s = mpuRead();

  // Accel-only tilt
  float roll_acc  = atan2f(s.ay, s.az);
  float pitch_acc = atan2f(-s.ax, sqrtf(s.ay*s.ay + s.az*s.az));

  // Complementary filter (yaw will drift without mag)
  const float alpha = 0.98f;
  roll  = alpha * (roll  + s.gx * dt) + (1.0f - alpha) * roll_acc;
  pitch = alpha * (pitch + s.gy * dt) + (1.0f - alpha) * pitch_acc;
  yaw   = yaw + s.gz * dt;

  // Fill Imu msg
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

  // Rough covariances (tune later)
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


// ======================================== Setup Function ========================================
// This function runs once when the ESP32 starts or resets. 
// It’s used to initialize the device: set up connections, configure pins, start communication, etc.
void setup() {
  Serial.begin(115200);
  // Gives the ESP32 and Serial connection a bit of time to stabilize before doing anything heavy.
  delay(500);

// ============ Setup micro-ROS transport over Wi-Fi ==================
// This is the key line where our ESP32 connects to the micro-ROS agent over Wi-Fi.
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);

  // This gives the ESP32 enough time to establish a stable Wi-Fi connection and initialize ROS transport.
  // Without this, some micro-ROS operations might start before the network is ready.
  delay(2000);

// === Configure Motor Pins ===
// The ESP32 needs to know which pins it can safely control.
// If we don’t set them as OUTPUT, the ESP32 might default to INPUT mode or behave unpredictably or not move the motors at all
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT);

// === Setup PWM Channels ===
  // Sets up PWM channel 0 with:
  // 5000 Hz → frequency of the signal (how fast the ON/OFF switches). 
  // 8 bits of resolution → gives you values from 0 to 255 for speed

  // This channel will later control the left motor’s speed.
  ledcSetup(0, 5000, 8);

  //Same setup, but for PWM channel 1. Will be used for the right motor’s speed
  ledcSetup(1, 5000, 8);


  // Connects our left motor’s PWM pin (e.g., GPIO 14) to PWM channel 0 . 
  // So when we call ledcWrite(0, speed);, it changes the left motor speed
  ledcAttachPin(L_PWM, 0);

  // Connects your right motor’s PWM pin (e.g., GPIO 17) to PWM channel 1. 
  // Now we can use ledcWrite(1, speed); for the right motor
  ledcAttachPin(R_PWM, 1);



  // I²C init for MPU-6050 (SDA=21, SCL=22 @ 400k)
  Wire.begin(21, 22, 400000);
  mpuInit();                 // [MOD]
  mpuCalibrate(600);         // [MOD] keep robot still during this

// ================ Initialize micro-ROS Node =============
// We need this when setting up ROS entities (like nodes, publishers, subscribers).
// This line gets the default memory manager used by micro-ROS.
  allocator = rcl_get_default_allocator();
  
// This initializes a support structure, which is like a starter kit that micro-ROS needs to run.
// support holds information about the ROS environment, memory, and communication setup.
// in simple terms "Prepare micro-ROS with everything it needs to start: memory, context, and communication."
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

// This creates a ROS node called "esp32_diffdrive".
// The empty string "" means it's not inside a namespace.
  RCCHECK(rclc_node_init_default(&node, "esp32_diffdrive", "", &support));

// ================== Setup cmd_vel Subscriber and IMU publisher==================
// The PC publishes a Twist message to the topic /cmd_vel
// Our ESP32 receives this message because it subscribed to cmd_vel
// The IMU sensor publishes the data as well to /imu/data

  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Initialize message memory if needed (Twist has no dynamic sequences/strings)
  // cmd_vel_msg = geometry_msgs__msg__Twist__init();

// [MOD] IMU publisher on /imu/data
  RCCHECK(rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data"));

  // [MOD] Set frame_id once: "imu_link"
  imu_msg.header.frame_id.data = (char*)"imu_link";
  imu_msg.header.frame_id.size = strlen("imu_link");
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

  // [MOD] 100 Hz IMU timer
  const unsigned int period_ms = 10;
  RCCHECK(rclc_timer_init_default(
    &imu_timer, &support, RCL_MS_TO_NS(period_ms), imuTimerCallback));

  // [MOD] Executor now handles 1 sub + 1 timer (size: 2)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
}


// ============================= Loop Function =============================

// In Arduino (and PlatformIO), loop() runs repeatedly after the setup() function finishes. it Keeps our robot actively listening to the cmd_vel topic from ROS 2.

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}
