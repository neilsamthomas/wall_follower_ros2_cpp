# ROS2 Robot Behavior Package

This ROS2 package contains three main components designed to control a robot's movement and record its odometry:

1. **wall_finder.cpp**: A service server that directs the robot to the nearest wall when called.
2.  **wall_follower.cpp**: Implements a wall-following behavior that makes the robot follow the wall on its right-hand side. It includes a ROS service client to call the service server in `wall_finder.cpp` and a ROS action client to call the action server in `odom_recorder.cpp`.
3. **odom_recorder.cpp**: An action server that starts recording the robot's odometry when called.

https://github.com/user-attachments/assets/812097b3-c2da-4191-b44b-e9fc886ed6f0
