#!/bin/bash

BUILD_TYPE="Release"
if [ $# -ge 1 ]; then
    BUILD_TYPE=$1
fi

# export MAKEFLAGS="-j 8" && \
colcon build \
    --symlink-install \
    --base-paths src/ \
    --packages-ignore \
        moveit_resources_panda_description \
        moveit_resources_pr2_description \
        moveit_resources_fanuc_description \
        moveit_resources_fanuc_moveit_config \
        moveit_resources_panda_moveit_config \
        moveit_resources \
        moveit_setup_assistant \
        pilz_industrial_motion_planner \
        moveit_ros_benchmarks \
        moveit_ros_robot_interaction \
        moveit_ros_visualization \
    --packages-up-to \
        moveit_ros_warehouse \
        moveit_ros_planning \
        moveit_ros_move_group \
        moveit_ros_planning_interface \
        moveit_visual_tools \
        moveit_msgs \
        moveit_plugins \
        moveit \
        pick_ik \
        bio_ik \
        trac_ik_kinematics_plugin \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF

[[ $? -eq 0 ]] && source install/setup.bash
