# Small Autonomous Mobile Robot
## For the past two years, I have been working in the field of autonomy and decided to build a DIY small autonomous mobile car.
#### The first challenge is budgeting. I have already designed an architecture for a teleoperated car using ROS2 on a Jetson Orin NX, which is straightforward since the Jetson provides a powerful computing platform for compiling and executing all necessary code.To reduce costs, I opted to use a Raspberry Pi Pico as my main microcontroller, which costs only about €4 instead of €800. The main limitation of the Raspberry Pi Pico is its lack of direct support for ROS2. However, it does support micro-ROS. Therefore, I decided to implement all the code within the micro-ROS framework.

# Here I have explained 
# Step 1: Install ROS 2 and micro-ROS Build System
### ROS2 installation: Please refer to the following website: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html for installing ROS2 humble.
### Micro-Ros installation: 
``bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

