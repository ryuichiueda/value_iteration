#!/bin/bash -evx

rm -f '/tmp/value_t=3.pgm' '/tmp/action_t=41.ppm'


source ~/.bashrc
source ~/catkin_ws/devel/setup.bash
roslaunch value_iteration test.launch || echo Finish
[ "$(md5sum < ~/catkin_ws/src/value_iteration/test/value_t=3.pgm)" = "$(md5sum < /tmp/value_t=3.pgm)" ]
[ "$(md5sum < ~/catkin_ws/src/value_iteration/test/action_t=41.ppm)" = "$(md5sum < /tmp/action_t=41.ppm)" ]
