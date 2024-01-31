#! /bin/bash

source /root/.bashrc;
sleep 2
python3 /root/catkin_ws/src/my_navigation_configs/python/singlerun.py --world $1 $2 $3 2> >(grep -v -E 'TF_REPEATED|buffer_core.cpp|^$');

