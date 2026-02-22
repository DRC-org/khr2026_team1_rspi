#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch khr2026_team1_simulation simulation.launch.py use_sim_data:=true map_mode:=true
