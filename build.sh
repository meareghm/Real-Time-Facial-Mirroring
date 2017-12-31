#!/bin/bash
catkin_make
cd src
pip3 install -t ../devel/lib/python2.7/dist-packages/ ./blender_api_msgs
cd ..
source devel/setup.bash

