# Small Autonomous Mobile Robot
## For the past two years, I have been working in the field of autonomy and decided to build a DIY small autonomous mobile car.
### The first challenge is budgeting. I have already designed an architecture for a teleoperated car using ROS2 on a Jetson Orin NX, which is straightforward since the Jetson provides a powerful computing platform for compiling and executing all necessary code.To reduce costs, I opted to use a Raspberry Pi Pico as my main microcontroller, which costs only about €4 instead of €800. The main limitation of the Raspberry Pi Pico is its lack of direct support for ROS2. However, it does support micro-ROS. Therefore, I decided to implement all the code within the micro-ROS framework.

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

### The micro-ROS build system consists of four steps:

####    Create – Downloads required code repositories and cross-compilation toolchains.
##### Cross-compilation is the process of compiling code on one system (the host: Our Computer) to run on a different system (the target:here Raspberry Pi Pico) that may have a different architecture, operating system, or environment.

##### For example: You write and compile code on your Ubuntu PC (x86_64).The compiled binary is meant to run on a microcontroller (Raspberry Pi Pico).

### Since the microcontroller may lack the necessary resources (CPU, memory, OS) to run a full compiler, cross-compilation allows you to generate compatible binaries on your powerful development machine.

####    Configure – Selects the app, transport method, and agent details.
####    Build – Cross-compiles the selected app.
####    Flash – Installs the compiled binaries on the hardware.
