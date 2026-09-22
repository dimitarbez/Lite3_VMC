# Maintained Lite3_VMC fork

This checkout is pinned by the [Lite3 ROS workspace wrapper](https://github.com/dimitarbez/lite3-ros-sim/tree/feature/emotion-robot-integration). Use the wrapper's `lite3-noetic/Makefile` for the supported ROS Noetic and Gazebo workflow. The [emotion_bot_ros guide](src/emotion_bot_ros/README.md) documents the current adapter, mapper, safety bridge, topics, and verification targets.

The commands below are the upstream simulator baseline. Run them only within the configured Noetic container when isolating the original controller.



```shell
roslaunch gazebo_model_spawn gazebo_startup.launch
roslaunch gazebo_model_spawn model_spawn.launch rname:=lite3 use_xacro:=true use_camera:=false #start controller
rosrun examples example_lite3_sim
rosrun examples example_keyboard
```

