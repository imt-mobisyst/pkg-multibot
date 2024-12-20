# Simulation demos

In this README you'll see all the commands needed to run all the demos.

For each solution, multiple demos are implemented. There are 3 type of demos *(with different complexity levels)*:
1. A very simple **talker/listener** example (except for the namespacing)
2. Multiple turtlesims running in parallel : you can place a `goal_pose` in Rviz and one of the robots will be assigned to it
3. One with the stage simulator, with 3 robots in a warehouse environment. It perfectly illustrates the scenario explained
[here](../README.md#1-scenario) :
    - when you add a `clicked_point` in Rviz, packages start to spawn at specific places in the map, and the robots coordinate
    to bring the packages to the correct deposit spot depending on their color.
    - when you send a message on the `retrieve` topic, one of the robots retrieves a package from the correct deposit zone and
    brings it to the retrieval zone


## 1. Robot separation using namespaces

Namespacing allows to add a prefix before every node, topic, service... in a launchfile. That way, they allow to avoid conflicts between data from different robots.

### a. Turtlesim demo

To start the demo, you can use the following command, that will take care of starting everything (`turtle`, `operator` and `rviz`) :
```bash
ros2 launch multibot turtlesim_namespace_launch.py nb_robots:="3"
```

### b. Stage demo

To start the demo, you can use the following commands, that will take care of starting everything (`rviz`, `stage`, `controller` and `operator`) :
```bash
ros2 launch multibot rviz_launch.py config:=config/stage.rviz
ros2 launch multibot stage_namespace_launch.py
```

To start spawning packages, use the `Publish Point` button in rviz (it will toggle the package spawning).  
To retrieve a package with a specific color, run `ros2 topic pub /retrieve std_msgs/msg/String "{data: 'green'}" --once`
*(other colors: `yellow`, `blue`, `red`)*.


## 2. Multi DOMAIN_ID communication

We will be using the [domain_bridge](https://github.com/ros2/domain_bridge/blob/main/doc/design.md) library that allows us to run multiple nodes in the same OS process, in order to "bridge" topics/services/actions from one DOMAIN_ID to another one.

<div align="center"><img src="../docs/img/domain_bridge.png" width="850" title="Example for the domain_brige library"></div>

To install it : 
```bash
apt install ros-iron-domain-bridge
```

> **Note :** The library doesn't seem to be maintained any more : [last commit](https://github.com/ros2/domain_bridge/commit/64e34de40218909b91057c368c10d4ce584af612) on January 2nd 2023. Maybe have a look at [DDS Router](https://github.com/eProsima/DDS-Router) or try to reproduce how it works inside our own communication nodes.


### a. Simple test with a talker and a listener

We only need to create a configuration file, telling the library which topics/services/actions need to be transmitted, from which `ROS_DOMAIN_ID` and to which `ROS_DOMAIN_ID` *(see [talker_bridge_config.yaml](./config/domain_bridge/talker_bridge_config.yaml))*
To start the bridge, we use the following command :

```bash
ros2 run domain_bridge domain_bridge <path_to>/bridge_config.yaml
```

> We can run this command from any terminal, regardless of the `ROS_DOMAIN_ID`

In another terminal, start the `talker` node :

```bash
ROS_DOMAIN_ID=2 ros2 run demo_nodes_cpp talker
```

In another terminal, start the `listener` node :

```bash
ROS_DOMAIN_ID=3 ros2 run demo_nodes_cpp listener
```


### b. Test with multiple turtlesim on the same computer

Each "robot" will be linked to a unique domain ID *(`bot_domain_id`)*. In each domain ID, there will be a `turtlesim_node` and a `turtlesim_controller` (which will control the movement of the turtle towards the goal poses).

To start the demo, you can use the following command, that will take care of starting everything (`turtle`, `operator` et `rviz`) :
```bash
ros2 launch multibot turtlesim_bridge_launch.py nb_robots:="3"
```
---

However, if you want to manually start the nodes yourself, here are the commands :

Starting the `turtle` in different terminals (nodes will be started with the correct `ROS_DOMAIN_ID` given as an argument):

```bash
ros2 launch multibot turtlesim_bridge_robot_launch.py bot_domain_id:="10" operator_domain_id:="1"
ros2 launch multibot turtlesim_bridge_robot_launch.py bot_domain_id:="11" operator_domain_id:="1"
```

Bridge nodes will be started by the launchfiles to transmit the topics needed by nodes running in the `operator_domain_id`. Then, we'll start the operator node (which is responsible for managing priority between turtles) in this domain :

```bash
ROS_DOMAIN_ID=1 ros2 run multibot static_operator.py --ros-args -p nb_robots:=2
```

Running `rviz` in the operator's `ROS_DOMAIN_ID` :
```bash
ROS_DOMAIN_ID=1 ros2 launch multibot rviz_turtlesim_launch.py
```

---
In order to send goal points for the turtles to go to, you can press the `2D Goal Pose` in Rviz or run the following command :
```bash
ROS_DOMAIN_ID=1 ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{pose: {position: {x: 9, y: 9.0, z: 0.0}}}" --once
```



### c. Test with the stage simulator

We'll use the following domain IDs :
- 0,1,2... for the different robots
- 99 for the operator/rviz
- 100 for the simulation

Start Rviz in the correct domain ID :
```bash
export ROS_DOMAIN_ID=99
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch multibot rviz_launch.py config:=config/stage.rviz
```

> **Note :** We use CycloneDDS here because there is a [bug](https://github.com/ros2/domain_bridge/pull/79) in the
> `domain_bridge` library that causes the `bidirectional` bridge configuration to create an infinite loop
> when using `rmw_fastrtps_cpp`

Launch the demo (with the simulator, the controllers, the nav2 stacks...):
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch multibot stage_bridge_launch.py
```

To start spawning packages, use the `Publish Point` button in rviz (it will toggle the package spawning).  
To retrieve a package with a specific color, run `ros2 topic pub /retrieve std_msgs/msg/String "{data: 'green'}" --once`
*(other colors: `yellow`, `blue`, `red`)*.

## 3. Network isolation with FastDDS Discovery server

DDS is the default protocol used by ROS2 for communicating between nodes. One aspect of this protocol is to look for elements that a node can communicate with on the newtwork. It's the "Discovery protocol".

In our use case, we'll use Fast DDS [Discovery server](https://docs.ros.org/en/iron/Tutorials/Advanced/Discovery-Server/Discovery-Server.html), which works similarly to a router and allows to isolate DDS subnets.


### a. Simple test with a talker and a listener

We are going to start multiple DDS Discovery servers :
- one to isolate a "local" network, which port is `11811`. This one will emulate one that would be on a robot computer.
- one on the common network between robots, which port is `11812`. This one will emulate one that would be on the operator computer.

```bash
fastdds discovery -i 0 -l 127.0.0.1 -p 11811 # Local
fastdds discovery -i 1 -l 127.0.0.1 -p 11812 # Shared
```

---


**Test 1 :** We check that a local `talker` node can be listened by a "local" node and a "common" node BUT not on a node on the operator
```bash
# Emulates a local node on the robot
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker
```
```bash
# Emulates a local node on the robot
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener
```
```bash
# Emulates a subnet node on the robot
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11812"
ros2 run demo_nodes_cpp listener
```
```bash
# # Emulates a node on the operator
export ROS_DISCOVERY_SERVER=";127.0.0.1:11812"
ros2 run demo_nodes_cpp listener
```

The first 2 `listeners` should receive the published messages, but not the "operator" node.

---

**Test 2 :** We check that a `talker` in the common network can be listened by a local node and also another node in the common network
```bash
# Emulates a subnet node on the robot
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11812"
ros2 run demo_nodes_cpp talker
```
```bash
# Emulates a local node on the robot
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener
```
```bash
# Emulates another subnet node on the robot
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11812"
ros2 run demo_nodes_cpp listener
```
```bash
# Emulates a node on the operator
export ROS_DISCOVERY_SERVER=";127.0.0.1:11812"
ros2 run demo_nodes_cpp listener
```

All 3 listeners should receive the published messages.

---


> **Note :** By default, the ROS2 CLI creates a node in order to listen for other nodes/topics in the network. For this to work with the DDS Discovery server architecture, we need to configure ROS2 as a **"Super client"**.  
This can be done thanks to the [super_client_config.xml](./config/dds_server/super_client_config.xml) configuration file and the following command :
> ```bash
> export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/super_client_config.xml
> ros2 daemon stop && ros2 daemon start # We restart the daemon to take the changes into account
> ```
> However, be careful because you will now have access to ALL nodes in the graph, **without any isolation of the network**.  
> In order to choose the discovery servers you want to connect to, you can comment the `<RemoteServer>` that don't interest you.  
> If you only want access to the **common network on the operator**, run the following commands :
> ```bash
> export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/super_client_operator_config.xml
> ros2 daemon stop && ros2 daemon start # We restart the daemon to take the changes into account
> ```


### b. Test with multiple turtlesim on the same computer

Each robot will host its own "local" DDS Discovery server, allowing communication between its internal nodes. Nodes that need to communicate with the outside elements (other robots/operator) will also connect to the operator's DDS Discovery server.


To start the demo, you can use the following command, that will take care of starting everything (`turtle`, `operator` et `rviz`) :
```bash
ros2 launch multibot turtlesim_dds_launch.py nb_robots:="3"
```

---

However, if you want to manually start the nodes yourself, here are the commands :

Starting the DDS Discovery servers :
```bash
fastdds discovery -i 0 -l 127.0.0.1 -p 11811 # Local DDS discovery server for robot 1
fastdds discovery -i 1 -l 127.0.0.1 -p 11812 # Local DDS discovery server for robot 1
fastdds discovery -i 2 -l 127.0.0.1 -p 11813 # Local DDS discovery server for robot 1
fastdds discovery -i 3 -l 127.0.0.1 -p 11814 # Common DDS discovery server, on the operator
```

> **Note :** Here, in this demo, we use different ports to emulate different machines. In reality, each robot will only host one local DDS Discovery server, and the last one will be hosted on the operator.


Starting the `turtle` (`turtlesim` et `turtle_controller`) in different terminals (nodes will be started connecting automatically to the correct DDS Discovery server(s)) :

```bash
ros2 launch multibot turtlesim_dds_robot_launch.py local_dds_server:="127.0.0.1:11811" subnet_dds_server:="127.0.0.1:11814" nb_robots:="3" robot_id:="1"
ros2 launch multibot turtlesim_dds_robot_launch.py local_dds_server:="127.0.0.1:11812" subnet_dds_server:="127.0.0.1:11814" nb_robots:="3" robot_id:="2"
ros2 launch multibot turtlesim_dds_robot_launch.py local_dds_server:="127.0.0.1:11813" subnet_dds_server:="127.0.0.1:11814" nb_robots:="3" robot_id:="3"
```

Starting the operator node :
```bash
export ROS_DISCOVERY_SERVER=";;;127.0.0.1:11814"
ros2 run multibot static_operator.py --ros-args -p nb_robots:=3
```

Starting rviz :
```bash
export ROS_DISCOVERY_SERVER=";;;127.0.0.1:11814"
ros2 launch multibot rviz_turtlesim_launch.py
```

---

In order to send goal points for the turtles to go to, you can press the `2D Goal Pose` in Rviz or run the following command :
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{pose: {position: {x: 9, y: 9.0, z: 0.0}}}" --once
```

### c. Test with multiple turtlesim on different computers *(pibot)*

First we need to install the `turtlesim` library on each *pibot* :
```bash
sudo apt install ros-iron-turtlesim
```

They will all be setup on `ROS_DOMAIN_ID=99`. The operator PC will have a static IP address (here it's *`10.89.5.90`*).

On the operator PC :
```bash
ros2 launch multibot pibot_dds_operator.py nb_robots:="2" common_dds_ip:="10.89.5.90" common_dds_port:="11811"
```

On each *pibot* :
```bash
ros2 launch multibot pibot_turtlesim_dds_launch.py nb_robots:=2 operator_server:="10.89.5.90:11811"
```

### d. Test with the stage simulator

All DDS servers will be hosted on the current machine, with IP `127.0.0.1`.
We'll use the following DDS server ports :
- 11811 for the simulation
- 11812 for the operator/rviz
- 11813,111814,11815... for the different robots

To launch rviz, run the following commands :
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/super_client_subnet_config.xml
ros2 daemon stop && ros2 daemon start
ros2 launch multibot rviz_launch.py config:=config/stage.rviz
```

In another terminal, launch the demo (with the simulator, the controllers, the nav2 stacks...):
```bash
ros2 launch multibot stage_dds_launch.py
```

To start spawning packages, use the `Publish Point` button in rviz (it will toggle the package spawning).  
To retrieve a package with a specific color, run `ros2 topic pub /retrieve std_msgs/msg/String "{data: 'green'}" --once`
*(other colors: `yellow`, `blue`, `red`)*.


## 4. Robot isolation using DDS partitions

DDS is the protocol used by ROS2 for communicating between nodes. DDS introduced a way to isolate DataWriters (Publishers)
and DataReaders (Subscribers) called [DDS partitions](https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/domain/domainParticipant/partition.html#partitions)

For a Publisher to communicate with a Subscriber, they have to belong at least to one common partition.

### a. Simple test with a talker and a listener

In a first terminal, run a first talker (in the `chatter1` topic):
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/talker_config.xml
ros2 run demo_nodes_cpp talker --ros-args -r chatter:=chatter1
```

In a second terminal, run a second talker (in the `chatter2` topic):
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/talker_config.xml
ros2 run demo_nodes_cpp talker --ros-args -r chatter:=chatter2
```

In a third terminal, run a first listener (to the `chatter1` topic):
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/listener_config.xml
ros2 run demo_nodes_cpp listener --ros-args -r chatter:=chatter1
```

In a last terminal, run a second listener (to the `chatter2` topic):
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/listener_config.xml
ros2 run demo_nodes_cpp listener --ros-args -r chatter:=chatter2
```

> Only the first listener should receive data, since the second one doesn't have a partition in common with the publisher.

---

All of the nodes and topics will be visible with `ros2 node list` and `ros2 topic list`

However, `ros2 topic echo <topic_name>` will only work on data published on the default partition (`""`). If you want to
echo topics published on other partitions, just use the configuration file that connects to every partition (`"*"`) except 
the default one (`""`) :
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/cli_config.xml
ros2 topic echo /chatter2
```


### b. Test with multiple turtlesim on the same computer

Each "robot" will have its own DDS partition (`robotX`). Their nodes will be communicating through topics with that partition. Topics that need to be shared across robots will be in the `shared` partition

To start the demo, you can use the following command, that will take care of starting everything (`turtle`, `operator` et `rviz`) :
```bash
ros2 launch multibot turtlesim_partition_launch.py nb_robots:="3"
```

---

However, if you want to manually start the nodes yourself, here are the commands :

Starting the `turtle` (`turtlesim` et `turtle_controller`) in different terminals (nodes will be started with the created config files to use their own partition `robotX` and the `shared` partition):
```bash
ros2 launch multibot turtlesim_partition_robot_launch.py nb_robots:="3" robot_id:="1"
ros2 launch multibot turtlesim_partition_robot_launch.py nb_robots:="3" robot_id:="2"
ros2 launch multibot turtlesim_partition_robot_launch.py nb_robots:="3" robot_id:="3"
```


Starting the operator node :
```bash
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/operator_config.xml
ros2 run multibot static_operator.py --ros-args -p nb_robots:=3
```

Starting rviz :
```bash
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/operator_config.xml
ros2 launch multibot rviz_turtlesim_launch.py
```



### c. Test with the stage simulator

We'll use the following DDS partitions :
- `robot_0`,`robot_1`,`robot_2`... for the different robots
- `shared` for the subnet (operator/rviz)

To launch rviz, run the following commands :
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/operator_config.xml
ros2 launch multibot rviz_launch.py config:=config/stage.rviz
```

In another terminal, launch the demo (with the simulator, the controllers, the nav2 stacks...):
```bash
ros2 launch multibot stage_partition_launch.py
```

To start spawning packages, use the `Publish Point` button in rviz (it will toggle the package spawning).  
To retrieve a package with a specific color, run `ros2 topic pub /retrieve std_msgs/msg/String "{data: 'green'}" --once`
*(other colors: `yellow`, `blue`, `red`)*.


## 5. Robot isolation with domain ID and Zenoh

### a. Simple test with a talker and a listener

First we will start the 2 Zenoh bridges, in 2 different terminals :
```bash
ROS_DOMAIN_ID=1 zenoh-bridge-ros2dds -c /path/to/bridge_config_talker.json5
ROS_DOMAIN_ID=2 zenoh-bridge-ros2dds -c /path/to/bridge_config_listener.json5
```

Then we will launch the talker and listener nodes, and prevent the DDS communication between nodes by
starting them in different domain IDs

```bash
ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp talker
ROS_DOMAIN_ID=2 ros2 run demo_nodes_cpp listener
```

### b. Test with the stage simulator

Each "robot" will have its own Zenoh bridge as well as its own namespace (`robotX`). Topics that need to be shared across robots will be specified in the Zenoh bridge `allow` configuration.

To launch rviz, run the following commands :
```bash
ROS_DOMAIN_ID=99 ros2 launch multibot rviz_launch.py config:=config/stage.rviz
```

In another terminal, launch the demo (with the simulator, the controllers, the nav2 stacks...):
```bash
ros2 launch multibot stage_zenoh_launch.py
```

To start spawning packages, use the `Publish Point` button in rviz (it will toggle the package spawning).  
To retrieve a package with a specific color, run `ros2 topic pub /retrieve std_msgs/msg/String "{data: 'green'}" --once`
*(other colors: `yellow`, `blue`, `red`)*.
