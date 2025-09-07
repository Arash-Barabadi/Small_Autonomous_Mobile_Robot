// ========================================= Includes =========================================
// 1. Include Libraries

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

// =============================== Compile-Time Transport Check ===============================
// 2.Checks if you're using the correct transport (serial). If not, compilation stops with an error.
// Ensures you don’t accidentally try to run this on Wi-Fi/UDP without modifying the code.

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

// How it checks?


// ============================== Declare Global ROS 2 Variables ==============================

rcl_publisher_t publisher;        

std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// ============================== Define Macros for Error Checking ==============================
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ================================ Define Error Loop ===========================================
// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}
// ================================ Timer Callback Function =====================================
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
// The timer is your alarm clock.
// The callback function is the job you do every time the alarm rings.
// The executor is like your assistant who checks if the alarm has rung and tells you to act.

// Gets called every time the timer "fires" (every 1000 ms as defined later). 
// Publishes msg to the topic.    Increments msg.data every time. So it sends: 0, 1, 2, 3...
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}
// ==================================== Setup Function ========================================== 
void setup() {
  // Configure serial transport
  // Starts the serial communication at 115200 baud. Configures the micro-ROS transport to use Serial. Waits 2 seconds for stability (important for ESP32 boot).
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);


// Fetches the default memory allocator used by the ROS 2 client libraries.
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

// ==================================== Loop Function ===========================================
void loop() {
  // Arduino main loop.
  // Every 100ms:
  // Runs the executor for up to 100 ms.
  // If the timer has elapsed, timer_callback() is called.
  // msg.data is published and incremented.

  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
