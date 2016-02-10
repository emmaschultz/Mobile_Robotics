# my_path_service

The client node, my_path_client.cpp, is hard coded with a set of poses that the 2D robot should take in order to navigate the maze into the upper left hard corner. The client sends over the list of subgoals/poses to the server node, my_path_service.cpp, which executes these subgoals in order. It does this by calculating the heading and distance for the robot to move. It does this until there are no more subgoals.

## Example usage
`roslaunch my_path_service my_path_service.launch`

`rosrun my_path_service my_path_client`
    
