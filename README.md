# Small Autonomous Mobile Robot

[🎥 Watch my project demo on YouTube](https://www.youtube.com/watch?v=UUFdapDHFw0)
<img width="300" height="400" alt="20251104_104700" src="https://github.com/user-attachments/assets/647b3a5f-ee14-4965-8a08-5e74b09f87aa" />


### The first challenge is budgeting. I have already designed an architecture for a teleoperated car using ROS2 on a Jetson Orin NX, which is straightforward since the Jetson provides a powerful computing platform for compiling and executing all necessary code.
### To reduce costs, I opted to use a ESP32 as my main microcontroller, which costs only about €5 instead of €800!  The main limitation of the ESP32 is its lack of direct support for ROS2. However, it does support micro-ROS. Therefore, I decided to implement all the code within the micro-ROS framework.

# 1st aim
## My first aim is to drive the robot over Wi-Fi communication between my laptop and the ESP32.I’ll use PlatformIO for building, testing, and deploying code on the ESP32, as it is a better alternative to the Arduino IDE. PlatformIO is installed on VS Code.
## Challenges accomplished
### 1- Originally, the robot continued moving indefinitely if no new /cmd_vel message was received (for example, when the Wi-Fi connection dropped or the teleop node stopped). This caused unsafe behavior — the last velocity command remained active and the robot kept accelerating or turning without control. To solve this, a command timeout watchdog was implemented. It continuously checks the time since the last received command and automatically stops both motors if no new command arrives within 250 ms as below:
```cpp
// =============== Update timestamp whenever a command arrives ========================
void cmdVelCallback(const void *msgin) {
  
  // at first update and record timestamp as soon as receiving a message
  last_cmd_us = micros();  // micros() returns the current time in microseconds

  //The incoming message is cast to a geometry_msgs::msg::Twist type
  const auto *msg = (const geometry_msgs__msg__Twist *)msgin;

  float v = constrain(msg->linear.x  / MAX_V, -1.0f, 1.0f);
  float w = constrain(msg->angular.z / MAX_W, -1.0f, 1.0f);

  float left_norm  = (v - w * TRACK / 2.0f) * K_LEFT;
  float right_norm = (v + w * TRACK / 2.0f) * K_RIGHT;

  setMotor(L_IN1, L_IN2, 0, speedToPwm(left_norm));
  setMotor(R_IN1, R_IN2, 1, speedToPwm(right_norm));
}
// ================== Check timeout each loop ======================
void checkCmdTimeout() {
  uint32_t now = micros();
  if (last_cmd_us && (now - last_cmd_us > CMD_TIMEOUT_US)) {
    safetyStop();
  }
}

