# value_iteration

a global planner for mobile robots based on value iteration


## install

This package works on ROS. The following procedure is for Ubuntu 18.04. It also works on Ubuntu 20.04.

```
$ sudo apt-get install ros-melodic-tf ros-melodic-map-server  <- change melodic to your ROS version
$ cd ~/catkin_ws/src                                          <- change to your workspace directory
$ git clone https://github.com/ryuichiueda/value_iteration.git
$ cd ~/catkin_ws
$ catkin_make
```

## test

```
$ ~/catkin_ws/src
$ ./test/test.bash
$ nautilus /tmp/
(There are many images.)
```
