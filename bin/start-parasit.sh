#!/usr/bin/env bash
cd `dirname $0`/..

echo "#    I N I T I A L I Z E   R O S   O N   S U B N E T"
source ../install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

echo "#    R U N   P A R A S I T"
ros2 run multibot_domid parasit
