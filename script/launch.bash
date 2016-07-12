#!/bin/bash

source /home/teresa/catkin_ws/devel/setup.bash
roslaunch teresa_driver teresa.launch > /dev/null 2>&1 &
echo $! 
