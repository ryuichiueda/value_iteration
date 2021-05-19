# value_iteration: value iteration package for ROS

a global planner for mobile robots based on value iteration

## Nodes

### vi_node

This node executes value iteration.

#### how to execute value iteration

The value iteration procedure rises up through the action `value_iteration/ViAction`, which is composed of the following goal, feedback, and result. An example of usage is shown in [vi_turtle_env.py](https://github.com/ryuichiueda/value_iteration/blob/main/scripts/vi_turtle_env.py).

* /vi_controller/goal
    * goal [geometry_msgs/PoseStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)
        * the destination
* /vi_controller/feedback
    * current_sweep_times (std_msgs/UInt32MultiArray)
        * the number of sweeps executed so far by each thread
    * deltas (std_msgs/Float32MultiArray)
        * the maximum change of state values caused by the latest sweep of each thread
* /vi_controller/result
    * finished (bool)
        * return true after the completion of value iteration

### vi_controller_turtle_env

This node receives the 2D Nav Goal from RViz and sends it to vi_node. It is implemented in `scripts/vi_turtle_env.py`.

#### Services

* policy ([grid_map_msgs::GetGridMap](http://docs.ros.org/en/kinetic/api/grid_map_msgs/html/srv/GetGridMap.html))
    * Provide calculated policy. The id of the optimal action is written as a float value in each cell.
* value ([grid_map_msgs::GetGridMap](http://docs.ros.org/en/kinetic/api/grid_map_msgs/html/srv/GetGridMap.html))
    * Provide calculated value function.
* action_list (not implemented)
    * Provide the id and velocity of every action.

#### Services Called

* static_map ([nav_msgs/GetMap](http://docs.ros.org/en/api/nav_msgs/html/srv/GetMap.html))
    * Initiate the map for localization.

## acknowledgement

This software is developped on the support of JSPS KAKENHI JP20K04382.

