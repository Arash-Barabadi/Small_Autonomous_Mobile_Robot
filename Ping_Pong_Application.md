
# Here we go:
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
microros_ws/firmware/src/uros/micro-ROS-demos/rclc/ping_pong
```

### The app has two publisher-subscriber pairs:

###    A ping publisher sends unique ping messages.
###    A ping subscriber listens for external pings.
###    A pong publisher responds when a ping is received.
