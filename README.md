# value_iteration: value iteration package for ROS

a global planner for mobile robots based on value iteration

## Nodes

### vi_node

This node executes value iteration.

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

