#!/bin/bash

# Source ROS, the MoveIt 2 workspace and plansys2 workspace
source /opt/ros/humble/setup.bash
source /root/ws_moveit/install/setup.bash
source /root/plansys2_ws/install/setup.bash
echo "Sourced ROS & MoveIt Humble"

# Execute the command passed into this entrypoint
exec "$@"
