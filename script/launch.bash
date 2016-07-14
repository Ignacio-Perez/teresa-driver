#!/bin/bash

source /home/teresa/teresa_catkin_pkgs/devel/setup.bash
roslaunch teresa_driver teresa.launch > /dev/null 2>&1 &
echo $! 
