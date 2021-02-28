#!/bin/bash -evx

source ~/.bashrc
source ~/catkin_ws/devel/setup.bash
roslaunch value_iteration test.launch || echo Finish
[ "$(md5sum < ~/catkin_ws/src/value_iteration/test/value_t=3.pgm)" = "$(md5sum < /tmp/value_t=3.pgm)" ]
