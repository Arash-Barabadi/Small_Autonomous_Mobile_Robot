# Small Autonomous Mobile Robot
## For the past two years, I have been working in the field of autonomy and decided to build a DIY small autonomous mobile car.
## The first challenge is budgeting. I have already designed an architecture for a teleoperated car using ROS2 on a Jetson Orin NX, which is straightforward since the Jetson provides a powerful computing platform for compiling and executing all necessary code.
## To reduce costs, I opted to use a Raspberry Pi Pico as my main microcontroller, which costs only about €4 instead of €800.
## The main limitation of the Raspberry Pi Pico is its lack of direct support for ROS2. However, it does support micro-ROS. Therefore, I decided to implement all the code within the micro-ROS framework.
