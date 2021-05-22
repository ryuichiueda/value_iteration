# value_iteration: value iteration package for ROS

a global planner for mobile robots based on value iteration

## Nodes

### vi_node

This node executes value iteration.

#### how to execute value iteration

The value iteration procedure rises up through the action `value_iteration/ViAction`, which is composed of the following goal, feedback, and result. An example of usage is shown in [vi_turtle_env.py](https://github.com/ryuichiueda/value_iteration/blob/main/scripts/vi_turtle_env.py).

* /vi_controller/goal
    * goal ([geometry_msgs/PoseStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html))
        * the destination
* /vi_controller/feedback
    * current_sweep_times (std_msgs/UInt32MultiArray)
        * the number of sweeps executed so far by each thread
    * deltas (std_msgs/Float32MultiArray)
        * the maximum change of state values caused by the latest sweep of each thread
* /vi_controller/result
    * finished (bool)
        * return true after the completion of value iteration

#### Services

* policy ([grid_map_msgs/GetGridMap](http://docs.ros.org/en/kinetic/api/grid_map_msgs/html/srv/GetGridMap.html))
    * Provide calculated policy. The id of the optimal action is written as a float value in each cell.
* value ([grid_map_msgs/GetGridMap](http://docs.ros.org/en/kinetic/api/grid_map_msgs/html/srv/GetGridMap.html))
    * Provide calculated value function.
* action_list (not implemented)
    * Provide the id and velocity of every action.

#### Services Called

* static_map ([nav_msgs/GetMap](http://docs.ros.org/en/api/nav_msgs/html/srv/GetMap.html))
    * Initiate the map for localization.

#### Subscribed Topics

* /mcl_pose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
    * the poes of the robot on the map; received only when the parameter `online` is true

#### Published Topics

* /cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html))
    * the control order to the robot; published only when the parameter `online` is true


#### Parameters

* action_list
    * Currently, `vi_node` requires an action list like the following. `onestep_forward_m` and `onestep_rotation_deg` mean the forward and rotational velocities respectively. `vi_node` calculates the optimal value function and the oplimal policy based on this action list.

```
  <rosparam>
    vi_node:
      action_list:
        - name: forward
          onestep_forward_m: 0.3
          onestep_rotation_deg: 0.0
        - name: back
          onestep_forward_m: -0.2
          onestep_rotation_deg: 0.0
        - name: right
          onestep_forward_m: 0.0
          onestep_rotation_deg: -20.0
        - name: left
          onestep_forward_m: 0.0
          onestep_rotation_deg: 20.0
  </rosparam>
```

* ~online (bool, defalut: false)
    * flag for using vi_node as a real-time planner
* ~theta_cell_num (int, default: 60) 
    * number of intervals of the discrete state space on theta-axis
* ~thread_num (int, default: 4) 
    * number of threads used on value iteration
* ~safety_radius (double, default: 0.2[m]) 
    * distance that should be kept between the center of the robot and an occupancy grid 
* ~safety_radius_penalty (double, default: 30[s], max: 1,000,000,000[s]) 
    * immediate penarty (negative immediate reward in the field of reinforcement learning) when the robot invades the safety radius. 
* ~goal_margin_radius (double, default: 0.2[m]) 
    * radius of the goal on xy-plane
* ~goal_margin_theta (int, default: 10[deg]) 
    * radius of the goal on theta-axis
* ~map_type (string, default: occupancy) 
    * choice of map for setting immediate costs and occupancy

#### Maps

This node uses an accupancy grid map. In `launch/vi_turtle_online.launch`, you can find the following line.

```
  <arg name="map_file" default="$(find turtlebot3_navigation)/maps/map.yaml"/>
```

TODO: or you can use a cost map, in which immediate cost of each cell is written in units of seconds. Cells given 255 as their immediate costs are regarded as occupancy cells.


### vi_controller_turtle_env

This node receives the 2D Nav Goal from RViz and sends it to vi_node. It is implemented in `scripts/vi_turtle_env.py`.

## Notes

### What's the cost means?

It is the sum of costs from a pose (position + orientation) to a point of goal. The cost means the time. Therefore, it means the time required to reach the goal if immediate penarties don't exist. As shown in the parameter list, we can define some kinds of immediate penalties, which are quantified in units of seconds. 

## acknowledgement

This software is developped on the support of JSPS KAKENHI JP20K04382.

