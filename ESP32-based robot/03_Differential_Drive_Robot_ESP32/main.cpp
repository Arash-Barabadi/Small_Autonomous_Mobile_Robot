// ============================= Standard and micro-ROS Libraries =============================
#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>


// ============================= ROS 2 Libraries =============================
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>


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

// ============================= ROS 2 micro-ROS Entities =============================
//You're preparing your robot to receive commands from ROS 2. To do that, you need a few "tools" from micro-ROS to: 

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

// ============================= Error Checking Macros =============================
// This macro checks if a function (fn) was successful, and if not, it enters an infinite error loop.
#define RCCHECK(fn) { if ((fn) != RCL_RET_OK) error_loop(); }

// This macro calls a function and stores the result, but it doesn’t check for errors or take action. It's used for non-critical operations.
#define RCSOFTCHECK(fn) { rcl_ret_t _rc = (fn); (void)_rc; }

// ============================= Error Loop Function =============================
void error_loop() {
  while (true) delay(100);
}
// ============================= Set Motor Speed and Direction =============================

// Function to set motor direction and speed using two digital pins and one PWM channel
void setMotor(int in1, int in2, int pwm_chan, float speed) {

  // Calculate the PWM duty cycle: 
  // Take the absolute value of speed (so negative speed doesn’t break it),
  // Clamp it between 0 and 255 to stay within 8-bit limits,
  // Convert the result to an unsigned 8-bit integer (0–255)
  uint8_t duty = (uint8_t)constrain(abs(speed), 0, 255);
  
  // Set motor direction:

  // If speed is positive, motor moves forward (IN1 = HIGH, IN2 = LOW)
  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  
  
  // If speed is negative, motor moves in reverse (IN1 = LOW, IN2 = HIGH)
  } else if (speed < 0) {
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
  float v = msg->linear.x;
  float w = msg->angular.z;
  const float track = 0.2f; // wheel separation (m), adjust accordingly
  float left = (v - w * track / 2) * 255;
  float right = (v + w * track / 2) * 255;
  setMotor(L_IN1, L_IN2, 0, left);
  setMotor(R_IN1, R_IN2, 1, right);
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

// ================== Setup cmd_vel Subscriber ==================
// The PC publishes a Twist message to the topic /cmd_vel
// Our ESP32 receives this message because it subscribed to cmd_vel


  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Initialize message memory if needed (Twist has no dynamic sequences/strings)
  // cmd_vel_msg = geometry_msgs__msg__Twist__init();


// ================== Setup cmd_vel Subscriber =====================   

// In ROS (and micro-ROS), an executor is like a manager or dispatcher. It continuously checks if: 
//there is new data from subscriptions, or timers need to run, or services need to respond, etc.

//ESP32 then calls our cmdVelCallback() function
//This line initializes the executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

// This line adds our cmd_vel subscription to the executor, and says: “Whenever a new Twist message arrives, call this function: cmdVelCallback.”
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &cmd_vel_sub,
    &cmd_vel_msg,
    &cmdVelCallback,
    //ON_NEW_DATA	:Trigger the callback only when new data is received
    ON_NEW_DATA));
}

// ============================= Loop Function =============================

// In Arduino (and PlatformIO), loop() runs repeatedly after the setup() function finishes. it Keeps our robot actively listening to the cmd_vel topic from ROS 2.

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}
