#!/bin/bash

usage="usage: $(basename "$0") SETUP_FILE [PROTOCOL] [-h] -- analyze network trafic of ros2 nodes discovery messages

positional arguments:
    SETUP_FILE location setup.bash of ROS 2
    [optional] PROTOCOL : DOMAIN for domain ID, SERVER for DDS Discovery Server, PARTITION for DDS Partitions, ZENOH for Zenoh, otherwise namespaces

options:
    -h  show this help text"


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

# If second argument must be one of the communication methods (see USAGE)
PROTOCOL=${2}


echo "--------------------------------- SETUP ---------------------------------"

# Prepare environment
echo "Source ROS2"
source /opt/ros/iron/setup.bash
echo "Source ROS2 workspace :" ${SETUP_FILE}
source ${SETUP_FILE}



# Setup dump file for capture
mkdir -p data
DUMP_FILE="data/namespaces.pcapng"
if [[ ${PROTOCOL} == "SERVER" ]]
then
    DUMP_FILE="data/discovery_server.pcapng"
elif [[ ${PROTOCOL} == "DOMAIN" ]]
then
    DUMP_FILE="data/domain_id.pcapng"
elif [[ ${PROTOCOL} == "PARTITION" ]]
then
    DUMP_FILE="data/partitions.pcapng"
elif [[ ${PROTOCOL} == "ZENOH" ]]
then
    DUMP_FILE="data/zenoh.pcapng"
fi






# Time running
RUN_TIME=15

# Start capture
rm -f ${DUMP_FILE} > /dev/null 2>&1
tcpdump -G $((RUN_TIME + 2)) -W 1 -i any -nn -s 0 -w ${DUMP_FILE} > /dev/null 2>&1 &
TCPDUMP_PID=$!





echo "-------------------------------- RUNNING --------------------------------"

# Start the operator launchfile
if [[ ${PROTOCOL} == "SERVER" ]]
then
    echo -e "-> Running DDS Discovery Servers\n"
    ros2 launch communication_test warehouse_dds_operator.py &

elif [[ ${PROTOCOL} == "DOMAIN" ]]
then
    echo -e "-> Running DOMAIN ID bridge\n"
    ros2 launch communication_test warehouse_bridge_operator.py &


elif [[ ${PROTOCOL} == "PARTITION" ]]
then
    echo -e "-> Running DDS partitions\n"
    ros2 launch communication_test warehouse_partition_operator.py &


elif [[ ${PROTOCOL} == "ZENOH" ]]
then
    echo -e "-> Running Zenoh bridge\n"
    ros2 launch communication_test warehouse_zenoh_operator.py &

else
    # Run simple namespaces
    echo -e "-> Running namespaces\n"
    ros2 launch communication_test warehouse_namespace_operator.py &
fi





# Wait for tcpdump to finish and send ctrl-c to talker and listeners
sleep $RUN_TIME


echo "-------------------------------- ENDING ---------------------------------"
echo "Trying to kill programs"


# Ends ROS2 nodes
pkill -f communication_test > /dev/null 2>&1

if [[ ${PROTOCOL} == "SERVER" ]]
then
    pkill -f discovery > /dev/null 2>&1

elif [[ ${PROTOCOL} == "DOMAIN" ]]
then
    pkill -f bridge > /dev/null 2>&1

elif [[ ${PROTOCOL} == "ZENOH" ]]
then
    pkill -f zenoh-bridge-ros2dds > /dev/null 2>&1
fi



# END

sleep 1

echo "Traffic capture can be found in : ${DUMP_FILE}"

chmod -R 777 data