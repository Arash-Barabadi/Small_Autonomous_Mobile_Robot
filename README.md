# Small Autonomous Mobile Robot

### The first challenge is budgeting. I have already designed an architecture for a teleoperated car using ROS2 on a Jetson Orin NX, which is straightforward since the Jetson provides a powerful computing platform for compiling and executing all necessary code.
### To reduce costs, I opted to use a ESP32 as my main microcontroller, which costs only about €5 instead of €800!  The main limitation of the ESP32 is its lack of direct support for ROS2. However, it does support micro-ROS. Therefore, I decided to implement all the code within the micro-ROS framework.

# 1st aim
### My first aim is to drive the robot over Wi-Fi communication between my laptop and the ESP32.I’ll use PlatformIO for building, testing, and deploying code on the ESP32, as it is a better alternative to the Arduino IDE. PlatformIO is installed on VS Code.
### Challenges accomplished
#### 1- Originally, the robot continued moving indefinitely if no new /cmd_vel message was received (for example, when the Wi-Fi connection dropped or the teleop node stopped). This caused unsafe behavior — the last velocity command remained active and the robot kept accelerating or turning without control. To solve this, a command timeout watchdog was implemented. It continuously checks the time since the last received command and automatically stops both motors if no new command arrives within 250 ms.
