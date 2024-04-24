#!/bin/bash

usage="usage: $(basename "$0") SETUP_FILE [PROTOCOL] [-h] -- analyze network trafic of ros2 nodes discovery messages

positional arguments:
    SETUP_FILE location setup.bash of ROS 2
    [optional] PROTOCOL : SERVER for DDS, DOMAIN for domain ID, otherwise namespaces

options:
    -h  show this help text"

seed=42
while getopts ':h:' option; do
  case "$option" in
    h) echo "$usage"
       exit
       ;;
   \?) printf "illegal option: -%s\n" "$OPTARG" >&2
       echo "$usage" >&2
       exit 1
       ;;
  esac
done
shift $((OPTIND - 1))

# First argument must be setup.bash of ROS 2
SETUP_FILE=${1}

if [ -z ${SETUP_FILE} ]
    then
    echo "$usage"
    exit 2
fi

# If second argument is SERVER it uses Discovery Service
PROTOCOL=${2}

# Prepare environment
echo "Source ROS2"
source /opt/ros/iron/setup.bash
echo "source to file: " ${SETUP_FILE}
source ${SETUP_FILE}

# Dump file for capture
mkdir -p data
DUMP_FILE="data/namespaces.pcapng"
if [[ ${PROTOCOL} == "SERVER" ]]
then
    DUMP_FILE="data/discovery_server.pcapng"
    echo "Run in DDS mode"
elif [[ ${PROTOCOL} == "DOMAIN" ]]
then
    DUMP_FILE="data/domain_id.pcapng"
    unset ROS_DISCOVERY_SERVER
    echo "Run in DOMAIN_ID mode"
else
    unset ROS_DISCOVERY_SERVER
    echo "Run in namespaces mode"
fi

# Time running
RUN_TIME=15

# Start capture
rm -f ${DUMP_FILE} > /dev/null 2>&1
tcpdump -G $((RUN_TIME + 2)) -W 1 -i any -nn -s 0 -w ${DUMP_FILE} > /dev/null 2>&1 &
TCPDUMP_PID=$!

# Start talker in SERVER or SIMPLE mode
if [[ ${PROTOCOL} == "SERVER" ]]
then
    echo "Using DDS servers"
    ros2 launch communication_test turtlesim_dds_launch.py &

elif [[ ${PROTOCOL} == "DOMAIN" ]]
then
    echo "Using DOMAIN ID bridge"
    ros2 launch communication_test turtlesim_bridge_launch.py &

else
    # Run simple namespaces
    echo "Using namespaces"
    ros2 launch communication_test turtlesim_namespace_launch.py &
fi

# Spawn 50 listeners. They will be CLIENTS if ${PROTOCOL} is SERVER, else they will
# be simple participants
# for i in {1..50}
# do
#     ros2 run demo_nodes_cpp \
#         listener --ros-args --remap __node:=listener_${i} > /dev/null 2>&1  &
# done

# Wait for tcpdump to finish and send ctrl-c to talker and listeners
sleep $RUN_TIME
# kill -s SIGINT $(ps -C talker) > /dev/null 2>&1

echo "Trying to kill programs"
# kill -s SIGINT $(ps -C listener) > /dev/null 2>&1

# Ends ROS2 nodes
kill -s SIGINT $(pgrep turtle) > /dev/null 2>&1
kill -s SIGINT $(pgrep operator) > /dev/null 2>&1
kill -s SIGINT $(pgrep rviz) > /dev/null 2>&1
if [[ ${PROTOCOL} == "SERVER" ]]
then
    kill -s SIGINT $(pgrep discovery) > /dev/null 2>&1
    kill -s SIGINT $(pgrep -f turtlesim_dds_launch.py) > /dev/null 2>&1

elif [[ ${PROTOCOL} == "DOMAIN" ]]
then
    kill -s SIGINT $(pgrep bridge) > /dev/null 2>&1
    kill -s SIGINT $(pgrep -f turtlesim_bridge_launch.py) > /dev/null 2>&1
else
    kill -s SIGINT $(pgrep -f turtlesim_namespace_launch.py) > /dev/null 2>&1

fi


sleep 1

echo "Traffic capture can be found in: ${DUMP_FILE}"

chmod -R 777 data

# Make sure they are killed
# pkill talker
# pkill listener
