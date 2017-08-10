# !/bin/bash

KINECT_RUN=$(rosnode list | grep kinect2_points | wc -l)
if ! [ $KINECT_RUN -eq 3 ]; then
	roslaunch kinect2_bridge kinect2_bridge.launch &
fi

TRANSFORM_RUN=$(rosnode list | grep kinect2_transform)
if ! [ $TRANSFORM_RUN -eq 2 ]; then
	roslaunch ~/perls/src/pyrobots/static_transform.launch &
fi

roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true kinect:=true

