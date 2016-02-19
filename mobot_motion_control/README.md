# mobot_motion_control

This project utilizes an action server and client in order to move a robot in a scene. The lidar_alarm node, mobot_lidar_alarm.cpp, uses the lidar to check whether the robot is too close to an object in front of it. The action client, mobot_action_client.cpp, subscribes to the lidar_alarm topic in order to prevent crashing into a wall. The client attempts to go straight forward by sending goals to the action server, mobot_action_server.cpp. However, when the lidar alarm is activated, the client will cancel all current goals and instead send a goal to avoid whatever obstacle is in the way. The client then resumes sending goals to the server telling the robot to move straight forward. The action server executes the goals that the client gives it.

For some reason, the robot does not go straight when given a goal angle of 0. The robot's movement is also very choppy. I am unsure of why either of these issues occur. But the robot does adjust its path when there is an obstacle ahead of it. It just keeps running back into the wall it has just avoided because it does not drive straight (despite the angle = 0 radians).

## Example usage
`roslaunch gazebo_ros empty_world.launch`

`roslaunch mobot_urdf mobot_w_lidar.launch`

Drag in the starting pen to gazebo.

`rosrun mobot_motion_control mobot_lidar_alarm`

`rosrun mobot_motion_control mobot_action_server`

`rosrun mobot_motion_control mobot_action_client`
    
