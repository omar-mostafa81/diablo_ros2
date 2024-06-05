# diablo_ros2
Diablo robot source files with robot_localization package for sensor integration and path following controller

## Instructions
- Install Kiss-icp for the velodyne
- Install the /src files into your ros2_workspace, build, and source.
- Install the /misc files into your ros2_workspace
### Establish the control over the robot (using an SSH to the Raspberry Pi)

`ros2 run diablo_ctrl diablo_ctrl_node`

### Launch the full setup of the diablo robot. (Using an SSH to the Intel NUC)

`ros2 launch diablo_ctrl diablo_full_setup.launch.py`

### Start the path follower node - or any other node. (Using an SSH to the Intel NUC)
- Modify the node with the path you would like the robot to follow (Initially set as a circle)
- run `ros2 run diablo_ctrl path_follower.cpp`
