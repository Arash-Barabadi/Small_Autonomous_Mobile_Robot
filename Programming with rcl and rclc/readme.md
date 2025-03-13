## This section is about micro-ROS C API :rclc , which helps us write micro-ROS programs easily.

###    Similar to ROS2: The main building blocks (like publishers, subscribers, services, and timers) work the same way in micro-ROS as they do in regular ROS2.
###    Built on ROS2 Core: micro-ROS uses the same ROS2 client library (rcl) as standard ROS2.
###    rclc Makes It Easier: The rclc package doesn’t change how things work—it just provides helper functions to make writing micro-ROS programs simpler.
###    No Extra Layers: Unlike rclcpp (C++) or rclpy (Python), which add extra layers on top of rcl, rclc only introduces new features when absolutely necessary.
###    Executors: One new feature in rclc (in comparison to rcl) is the Executor, which helps manage multiple tasks efficiently.

## In short, rclc = micro-ROS version of rcl, but easier to use!
