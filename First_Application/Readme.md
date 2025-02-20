# Step 1: Install ROS 2 and micro-ROS Build System
### ROS2 installation: 
### Please refer to the following website: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html for installing ROS2 humble.
### Micro-Ros installation: 
### 1- Source the ROS 2 installation
```bash 
source /opt/ros/$ROS_DISTRO/setup.bash
```
### 2- Create a workspace and download the micro-ROS tools
```bash
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```

### 3- Update dependencies using rosdep
```bash
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
```

### 4- Install pip
```bash
sudo apt-get install python3-pip
```

### 5- Build micro-ROS tools and source them
```bash
colcon build
source install/local_setup.bash
```
# Step 2: Understand the micro-ROS Build System Workflow

## The micro-ROS build system consists of four steps:

###    1-Create – Downloads required code repositories and cross-compilation toolchains.
#### ***Cross-compilation is the process of compiling code on one system (the host: Our Computer) to run on a different system (the target:here Raspberry Pi Pico) that may have a different architecture, operating system, or environment.***

##### For example: You write and compile code on your Ubuntu PC (x86_64).The compiled binary is meant to run on a microcontroller (Raspberry Pi Pico).

### Since the microcontroller may lack the necessary resources (CPU, memory, OS) to run a full compiler, cross-compilation allows you to generate compatible binaries on your powerful development machine.

###    2-Configure – Selects the app, transport method, and agent details.
###    3-Build – Cross-compiles the selected app.
###    4-Flash – Installs the compiled binaries on the hardware.

# Step 3: Create a Firmware Workspace

## Create the firmware workspace:
```bash
ros2 run micro_ros_setup create_firmware_ws.sh host
```
### A new firmware/ folder is created, containing:

###    main.c: Application logic & CMakeLists.txt: Compilation script

# Step 4: Configure the Ping Pong Application

##    Locate the built-in ping_pong app:
```bash
microros_ws/src/uros/micro-ROS-demos/rclc/ping_pong
```

### The app has two publisher-subscriber pairs:

###    A ping publisher sends unique ping messages.
###    A ping subscriber listens for external pings.
###    A pong publisher responds when a ping is received.

# Step 5: Build the Firmware

## Since micro-ROS is running on Linux (not a microcontroller), no cross-compilation is needed. Build the firmware:
```bash
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash
```
# Step 6: Create and Build the micro-ROS Agent

##    Download the micro-ROS agent packages:
```bash
ros2 run micro_ros_setup create_agent_ws.sh
```
## Build the agent:
```bash
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

### What is the micro-ROS Agent? 🤖📡

#### The micro-ROS Agent is like a translator or middleman between micro-ROS nodes (running on tiny devices or a Linux system) and the main ROS 2 network.
#### 🔹 Role of micro-ROS Agent

####    Microcontrollers (tiny computers) can't directly communicate with a full ROS 2 system.
####    The micro-ROS Agent acts as a bridge between:
####        micro-ROS nodes (running on embedded devices or Linux).
####        ROS 2 nodes (running on a standard computer).
####    It receives messages from micro-ROS nodes and forwards them to ROS 2, and vice versa.

#### 💡 Think of it like an interpreter 🗣️
#### Imagine a person speaking English (ROS 2) and another speaking Spanish (micro-ROS). The micro-ROS Agent is the interpreter who helps them understand each other.


# Step 7: Run the micro-ROS Application

##    Start the micro-ROS agent:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
## Open another terminal and start the micro-ROS node:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_microxrcedds
ros2 run micro_ros_demos_rclc ping_pong
```

# Step 8: Test the Application
##  8.1 Verify the Ping Messages

##  Now, we want to check that everything is working. Open a new command line. We are going to listen to the ping topic with ROS 2 to check whether the micro-ROS Ping Pong node is correctly publishing the expected pings:

```bash
ros2 topic echo /microROS/ping
```

##  Expected output(we should see the topic messages published by the Ping Pong node every 5 seconds):
```bash
    stamp:
      sec: 20
      nanosec: 867000000
    frame_id: '1344887256_1085377743'
    ---
```
## At this point, we know that our app is publishing pings. 
## Let’s check if it also answers to someone else’s pings. If this works, it’ll publish a pong. So, first of all, let’s subscribe with ROS 2 to the pong topic from a new shell (notice that initially we don’t expect to receive any pong, since none has been sent yet):


##    Open a new terminal and listen to the /microROS/pong topic:
```bash
ros2 topic echo /microROS/pong
```
##  8.2 And now, let’s publish a fake_ping with ROS 2 from yet another command line and Verify Pong Response:
```bash
ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: "fake_ping"}'
```
## Now, we should see this fake_ping in the ping subscriber console, along with the micro-ROS pings:
```bash
user@user:~$ ros2 topic echo /microROS/ping
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
stamp:
  sec: 305
  nanosec: 973000000
frame_id: '451230256_1085377743'
---
stamp:
  sec: 310
  nanosec: 957000000
frame_id: '2084670932_1085377743'
---
```
## Also, we expect that, because of having received the fake_ping, the micro-ROS node will answer with a pong:
```bash
user@user:~$ ros2 run micro_ros_demos_rcl ping_pong
Ping send seq 1706097268_1085377743
Ping send seq 181171802_1085377743
Ping send seq 1385567526_1085377743
Ping send seq 926583793_1085377743
Ping send seq 1831510138_1085377743
Ping received with seq fake_ping. Answering.
Ping send seq 1508705084_1085377743
Ping send seq 1702133625_1085377743
Ping send seq 176104820_1085377743
```
## As a consequence, in the pong subscriber console, we should see the micro-ROS app answer to our fake_ping:
```bash
user@user:~$ ros2 topic echo /microROS/pong
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
```
# Step 9: Run Multiple Ping Pong Nodes

##    No Extra Hardware Required – Unlike embedded systems, where each micro-ROS node would typically run on a separate microcontroller or board, Linux allows you to simulate multiple micro-ROS nodes on the same machine.

##    Multi-Node Testing – You can run multiple instances of the ping_pong application in different terminals, each acting as an independent micro-ROS node. This helps simulate and test real-world scenarios where multiple nodes communicate.

##    Using the Same micro-ROS Agent – Since micro-ROS nodes need an agent to communicate with ROS 2, all instances can share the same running agent, reducing setup complexity.

##    Practical Testing Method – By opening four different terminals, each running a separate instance of the ping_pong node, you can observe how they interact with each other in a simulated multi-node environment.

##    What Happens in Practice?

###    When multiple instances of the ping_pong node run:

####        1-Each instance sends pings with a unique identifier.
####        2-Other instances respond with pongs, mimicking communication between independent micro-ROS devices.
####        3-This allows for debugging and performance evaluation without needing multiple physical devices.
