# my_lidar_alarm

The my_lidar_alarm.cpp file works by outlining a "corridor" of set width in front of the robot. If anything intersects with this corridor, then the alarm is triggered and is published. This alarm is then picked up by the reactive_commander.cpp which stops the robot from colliding with the obstruction and turns the robot until that obstruction is no longer intersecting with the corridor. The robot then proceeds forward again.

## Example usage
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`

`rosrun my_lidar_alarm my_lidar_alarm`

`rosrun reactive_commander reactive_commander`
