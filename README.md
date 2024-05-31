# pkg-multibot

This repository's goal is to study the **coordination of a robot fleet** using **ROS2** (Iron). We will study different communication methods and architectures to achieve a specific scenario, and compare them based on different criteria.

Although multi-robots research has been a subject for many years now (late 80s), there is still no standard on how to
architecture communication between them using ROS2. This README's goal is to show the main communications methods that
exist, their advantages and drawbacks, so that you can choose the one that best fits your needs.

## Table of contents

1. [Scenario](#1-scenario)
1. [Comparison criteria](#2-comparison-criteria)
1. [Communication methods](#3-communication-methods)
    - [Namespacing](#namespacing)
    - [Different domain IDs](#different-domain-ids)
    - [DDS Discovery servers](#dds-discovery-servers)
    - [DDS Partitions](#dds-partitions)
1. [Comparing the communication methods](#4-comparing-the-communication-methods)
1. [Comparing the different architectures](#5-comparing-the-different-architectures)
2. [Quality of service options](#6-quality-of-service-options)
3. [References](#7-references)


## 1. Scenario

We want to be able to control an **heterogenous fleet of robots** (for example robots from different vendors) in an 
industrial environment.

We will consider the following situation :  
In a warehouse, there are **2 arrivals of packages**. At random time intervals, packages arrive at each arrival zone.
Each package has a **specific color**, and for each color there is a corresponding **deposit zone** in the map.

<div align="center"><img src="docs/img/warehouse-scenario-map.png" width="500" title="Scenario warehouse map"></div>

We will consider 2 possible **tasks** for the robots :
- **Store :** Once a package arrives at a pickup spot, a robot will be selected to pick up the package, and carry it to the correct deposit zone, **depending on its color**.
- **Retrieve :** An operator can send a request to retrieve a package from a specific color. A robot will be selected to go to the correct storing position, and bring back a package to the **retrieval zone**

There will be 3 robots working together, **coordinating**, in order to achieve these tasks in the shortest time possible.

This coordination will be achieved thanks to an **auction/bid system**, that would assign the task to the best robot (based
on its position and its waypoints queue). This auction system could either be centralized, with an entity listening all
the bids and choosing the best one, or distributed, with each robot comparing its bid with the others.

First we'll consider that once a task is assigned to a robot, it can't abandon it and give it to another robot. However,
we could later add an **intermediate zone**, where robots would drop the package they are currently carrying to move to
another task, and another robot would be assigned that package, if that solution is globally better for the fleet.

As a bonus, it would be interesting to see how well the architecture is able to adapt to new robots dynamically added to the
fleet or robot failures.
Robot failures would be simulated by sending a message containing the robot ID on a global topic.

First, we'll consider that all the robots evolve in a known map. At the end, it would also be interesting to study how the
fleet could share information to create a common map with multi robot SLAM algorithms.


## 2. Comparison criteria 

To compare the different methods and architectures, we'll use different criteria :

- **Dynamism :** Does the architecture allow to dynamically add a robot to the fleet ? (dynamic identification...)
- **Resilience :** Does the architecture continue to work when there are failures (of the robots or the operator) ? 
- **Reliability :** Are there losses in the communication ?
- **Isolation :** Are robot specific informations shared or kept local ? How much control do we have over the information shared ?
- **Network usage :** How much data is transfered on the network ?
- **Scalability :** Does the system still work well when there are lots of robots ?
- **Computability :** Does the system require a lot of computing power (both on the robots and the operator) ?
- **Ease of simulation :** How easy is it to reproduce this communication architecture in a simulation ?
- **Ease of programming :** Does this architecture require the programmer to make a lot of configuration on each robot to allow them to communicate ?
- **Ease of debugging :** Is it simple to see the list of nodes / topics on a specific robot, and listen to the data published ?


> [!NOTE]
> When no node are subscribed to a topic, the data is not sent on the network. That way, in the `Network usage` criteria we'll
> only consider the network traffic during the **discovery** of nodes.
> 
> The discovery protocol is the automatic finding and matching of publishers and subscribers across nodes so they can start 
> sharing data. It is part of the underlying communication protocol of ROS2 (DDS: Data Distribution Service)

## 3. Communication methods

Here is the list of the different communication methods we will study :

- **Namespacing**
- **Different domain IDs**
- **DDS Discovery servers**
- **DDS Partitions**

### Namespacing
> See working demos [here](communication_test/README.md#1-robot-separation-using-namespaces)

**Namespaces** are prefixes to node names, topics, actions and services. They allow to have multiple elements with the same name but different prefix. 

In a multi-robot scenario, namespacing is the easiest solution to seperate each robot with a unique namespace, in order for robots
to not have name conflicts when running the same nodes and using the same topics.
> An example of this is to prefix the `cmd_vel`
topic for robots (`robot1/cmd_vel` and `robot2/cmd_vel`), to prevent them from having the same velocity command.

With our multi-robot architecture, we would have a configuration like the following :
<div align="center"><img src="docs/img/namespacing_architecture.png" width="850" title="Namespacing architecture example"></div>


### Different domain IDs
> See working demos [here](communication_test/README.md#2-multi-domain_id-communication)

ROS2 uses **DDS** (**D**ata **D**istribution **S**ervice) as the default middleware for communication. DDS allows nodes to 
discover other nodes that are on the same network. In order to create different logical networks, DDS provides a feature called 
the **domain ID**. Each node is allowed to communicate to nodes that are on the same ID, but can't communicate with nodes on other domain IDs.

In ROS2, the default domain ID is 0, but it can be configured using the `ROS_DOMAIN_ID` env variable (between 0 and 101 inclusive). The domain ID is then mapped to a UDP port, thus creating application isolation.

In a multi-robot scenario, assigning a different `ROS_DOMAIN_ID` to each robot allows to completely isolate them from the others. 
However, using the [domain_bridge](https://github.com/ros2/domain_bridge/blob/main/doc/design.md) library, we can create a bridge
between different domain IDs, and specify which topics should be broadcasted towards another domain ID (which would be shared between robots).

This library allows us to run multiple nodes in the same OS process, in order to share data and "bridge" topics/services/actions from one DOMAIN_ID to another one.
<div align="center"><img src="docs/img/domain_bridge.png" width="850" title="Example for the domain_brige library"></div>


With our multi-robot architecture, we would have the following configuration :
<div align="center"><img src="docs/img/domain_id_architecture.png" width="850" title="Multi domain ID architecture example"></div>


### DDS Discovery servers
> See working demos [here](communication_test/README.md#3-network-isolation-with-fastdds-discovery-server)

As stated before, DDS is the protocol used by ROS2 for communicating between nodes. One aspect of this protocol is to look for
elements that a node can communicate with on the newtwork. It's the "Discovery protocol". By default, the **Simple Discovery 
protocol** is used, which consists in sending multicast messages to every other node in the network.

Fast DDS, one of the DDS middlewares, provides a [Discovery server](https://docs.ros.org/en/iron/Tutorials/Advanced/Discovery-Server/Discovery-Server.html) to replace the **Simple Discovery protocol**. It works similarly to a router and
allows to isolate DDS subnets.
Each node can choose which DDS Discovery servers (it can be more than 1) it connects to using the `ROS_DISCOVERY_SERVER` env 
variable. Its main purpose is to reduce the network traffic induced by the discovery phase.

<div align="center"><img src="https://docs.ros.org/en/iron/_images/ds_partition_example.svg" width="850" title="DDS Network isolation example"></div>

> Listener 1 discovers topics from Talker 1 & 2 but Listener 2 only discovers topics from Talker 1

A discovery server is described by :
- its **IP address**
- its **port**
- its **ID**
> [!NOTE] 
> This solution is similar to the *"multi-master"* solution that existed in ROS1, as we're using IP addresses to connect
> multiple robots

In our multi-robot scenario, we could use this Discovery server to isolate nodes running on the robot, by connecting them to a DDS
Discovery server running locally. Nodes that also need to communicate to other robots would connect to both their local DDS server
and either a global one or another robot's one.


With our multi-robot architecture, we would have the following configuration :
<div align="center"><img src="docs/img/dds_architecture.png" width="850" title="FastDDS Discovery Server architecture"></div>

### DDS partitions
> See working demos [here](communication_test/README.md#4-robot-isolation-using-dds-partitions)


As stated before, DDS is the protocol used by ROS2 for communicating between nodes. DDS introduced the concept of
[**partitions**](https://docs.ros.org/en/iron/Tutorials/Advanced/FastDDS-Configuration.html#using-partitions-within-the-topic) :
each partition is defined by a name (**string**), and only elements that have a partition in common can communicate. 

Contrary to the DOMAIN_ID, nodes still receive the broadcast discovery messages (since they are on the same DOMAIN_ID they have
the same UDP port) but drop them if they don't have a partition in common.

Partitions can be applied to specific nodes, but also more precisely **publishers/subscribers** *(DataReaders/DataWriters in DDS 
terms)*. To configure this, you can create an **XML file** and apply it by setting the `FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/file.xml` env variable.

<div align="center"><img src="docs/img/dds_partitions.png" width="850" title="DDS partitions"></div>

In our multi-robot scenario, we could have **one partition for each robot** (`robot_X`). Topics that need to stay local would be 
published to that partition and topics that need to be shared across robots would be published in the `shared` partition.

With our multi-robot architecture, we would have the following configuration :
<div align="center"><img src="docs/img/dds_partitions_architecture.png" width="850" title="FastDDS Discovery Server architecture"></div>


### Others to check

- [Security](https://github.com/ros2/sros2/blob/master/SROS2_Windows.md) inside ROS2 to prevent communication if the correct
certificate is not given

- DDSrouter (+ HusarnetVPN)

- [DDS data_sharing](https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/common.html#xml-datasharing) QoS property

- [DDS domain tag](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/Content/UsersManual/ChoosingDomainTag.htm) (only CycloneDDS) that drops messages from the same DOMAIN_ID if nodes don't have the same domain tag
    > Is it possible for a node to be on multiple `tag` at the same time / switch when publishing ?


- Custom communication outside of ROS2 (but how to simulate) :
    > MQTT, Lora, Zigbee, Zenoh, ad hoc multi-hop with babel protocol

- Hubs ?? (see [this][1])



## 4. Comparing the communication methods

### Namespacing

- **Dynamism :** You can add robots but their namespace will not be set dynamically so you'll have to make sure yourself they are unique
- **Reliability :** No additional losses in the communication
- **Isolation :** Everything is shared to everyone in the network (other robots, the operator PC and any other PC)
- **Network usage :** At launch, every node tries to discover the nodes it can communicate with (Simple Discovery protocol) 
using a broadcast request. This creates a lot of network traffic. After that, only nodes that should communicate (publisher/
subscriber) will send packets to the network, thus not impacting the traffic.
- **Computability :** No additional nodes are needed for the communication
- **Ease of simulation :** Nothing special to configure, just make sure your namespaces match the simulator's.
- **Ease of programming :** Just add the namespace to everything started by a launchfile (1 line)
- **Ease of debugging :** Easy, just use the tools of ROS2 (`ros2 node list`, `ros2 topic list`, `rqt_graph`, `rviz`...)

### Different domain IDs

- **Dynamism :** You can add robots but their `ROS_DOMAIN_ID` will not be set dynamically so you'll have to make sure yourself 
they are unique
- **Reliability :** The QoS properties are "propagated" by the domain bridge, so the reliability is the same.
- **Isolation :** Only what is specified in the bridge configuration file is shared in the shared `ROS_DOMAIN_ID`. Every robot as
well as the operator can access that information.
- **Network usage :** At launch, every node tries to discover the nodes it can communicate (Simple Discovery protocol) using a 
broadcast request. Since the robots are fully isolated, the network traffic will be reduced. After that, only nodes that 
should communicate (publisher/subscriber) will send packets to the network, thus not impacting the traffic.
- **Computability :** Additional nodes are needed, to be able to bridge the informations from one domain ID to the other (2 nodes 
in a shared process for each bridge)
- **Ease of simulation :** By default, Gazebo (or any other simulator) runs in a specific `ROS_DOMAIN_ID`. That way, you can't 
have robots evolving in different domain IDs inside of Gazebo. You'll have to spawn them in the simulator's domain ID, and then
create bridges for the default topics (`scan`, `odom`...) to the corresponding domain IDs.
- **Ease of programming :** Set the correct environment variable before starting the nodes in the launchfile (1 line) and start 
the bridges in the launchfile (with a specific configuration file)
- **Ease of debugging :** To see the nodes/topics running on a specific robot, you must first export the `ROS_DOMAIN_ID` 
environment variable to the robot ID. Then you can use the ROS2 debug tools (`ros2 node list`, `ros2 topic list`, 
`rqt_graph`, `rviz`...).

> [!NOTE]
> There is a limited number of possible `ROS_DOMAIN_ID` values (see [this](https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html#choosing-a-domain-id-long-version)).
> If your fleet is really big (more than 100 robots), you might encounter domain ID collision between robots.

### DDS Discovery servers

- **Dynamism :** You can add robots dynamically as long as you know the IP address, port and ID of the shared DDS Discovery Server.
However, if you are in a distributed architecture, you'll need to know every other robots' IP.
- **Reliability :** No losses in the communication
- **Isolation :** Nodes connected only to the local DDS Discovery server will be isolated. In addition, nodes connected to both
the local DDS Discovery server and the operator's (="public" nodes) will be able to communicate to the shared network. Every
other public node from other robots as well as the operator can access that information. However, when these public nodes
will publish messages to local nodes, they will also be visible on the shared network and that can cause naming problems.
- **Network usage :** At launch, every node tries to discover the nodes it can communicate with. Since the networks are isolated, the network traffic will be reduced (see [this](communication_test/scripts/README.md)).
After that, only nodes that should communicate (publisher/subscriber) will send packets to the network, thus not impacting the traffic.
- **Computability :** Each robot need to host their own DDS Discovery server. There is also one hosted for the shared communication.
- **Ease of simulation :** You have to manually code bridges to send specific informations to the shared network. The lack
of isolation creates conflicts that require you to use namespaces.
- **Ease of programming :** Set the `ROS_DISCOVERY_SERVER` environment variable before starting the nodes in the launchfile 
(depending on whether they can communicate with the shared network or not).
Creating the `ROS_DISCOVERY_SERVER` environment variable takes multiple lines and is not easily readable since the IP address 
needs to be in the correct place in the list, based on the server ID *(example : `;;10.89.5.110:11811` for a server with ID=2)*
- **Ease of debugging :** By default, your ROS2 CLI will not have access to any information, even if the `ROS_DISCOVERY_SERVER` is 
correctly set. For it to work with the DDS Discovery server architecture, you need to configure ROS2 as a **"Super client"** (so 
that it discovers everything on the network). This can be done with an XML configuration file specifying the IP addresses, ports 
and IDs of the DDS Discovery servers you want to connect to. However, every time you change the configuration you'll have to
restart the ROS2 Daemon (`ros2 daemon stop && ros2 daemon start`) for it to be applied.


> [!NOTE]
> It seems possible to change the ROS_DISCOVERY_SERVER env variable at runtime (but only remove some of them from the list)
> (see the end of [this section](https://readthedocs.org/projects/eprosima-fast-rtps/downloads/pdf/latest/#subsection.7.26.4))  

### DDS partitions

- **Dynamism :** You can add robots and their local partition can be their IP address because they are unique
- **Reliability :** No losses in the communication
- **Isolation :** Only what is specified in the bridge configuration file is shared to the corresponding partitions. You can 
choose to send some topics only to the operator PC and some only to other robots.
- **Network usage :** To check.
- **Computability :** No additional nodes are needed for the communication
- **Ease of simulation :** To check. Bridges might be necessary, or at least a specific XML configuration before launching Gazebo.
- **Ease of programming :** Create a specific configuration file (XML) and set it as an environment variable before starting the 
nodes in the launchfile (1 line).
- **Ease of debugging :** By default, your ROS2 CLI will be able to see the list of topics (`ros2 node list`) on the network.
However, it will not have access (`ros2 topic echo`) to any information published to any partitions that is not the default
empty one (`""`). For it to work, you need to configure ROS2 to listen to these specific partitions. This can be done with an XML
configuration file, specifying the partitions that you want to subscribe to 
*(see [listener_config.xml](./communication_test/config/dds_partitions/listener_config.xml))*. If you want to subscribe to all
partitions, you can make a configuration file subscribing to the `"*"` paritition 
*(see [cli_config.xml](./communication_test/config/dds_partitions/debug/cli_config.xml))*, which corresponds to all partitions **except the default one**


> [!NOTE]
> In theory, DDS partitions can be changed dynamically at runtime
> (see [this](https://readthedocs.org/projects/eprosima-fast-rtps/downloads/pdf/latest/#subsubsection*.434)).
> This would allow the operator nodes to only send messages to specific robots if needed and also set their local partition
> dynamically by talking to the operator to get a unique ID.  
> However, ROS2 abstracts some of the configuration, and we can't do it anymore.
> You could maybe modify the DDS RMW implementations to get access to Publisher/Subscriber and modify partitions dynamically
> (see [this](https://discourse.ros.org/t/restricting-communication-between-robots/2931/31)) but it seems overcomplicated

### Results

Here are the results, based on the above explanations for each method.

<table style="margin-top:5px;">
    <thead>
        <tr style="border:none;">
            <th style="border: none;"></th>
            <th colspan="9" style="text-align:center">Criteria</th>
        </tr>
        <tr>
            <th align="center">Method</th>
            <th align="center">Dynamism</th>
            <th align="center">Reliability</th>
            <th align="center">Isolation</th>
            <th align="center">Network Usage</th>
            <th align="center">Computability</th>
            <th align="center">Ease of simulation</th>
            <th align="center">Ease of programming</th>
            <th align="center">Ease of debugging</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <th>Namespaces</th>
            <td align="center">🟠</td>
            <td align="center">✅</td>
            <td align="center">❌</td>
            <td align="center">🟠</td>
            <td align="center">✅</td>
            <td align="center">✅</td>
            <td align="center">✅</td>
            <td align="center">✅</td>
        </tr>
        <tr>
            <th>Domain ID</th>
            <td align="center">🟠</td>
            <td align="center">✅</td>
            <td align="center">✅</td>
            <td align="center">🟨</td>
            <td align="center">🟠</td>
            <td align="center">🟨</td>
            <td align="center">🟨</td>
            <td align="center">🟨</td>
        </tr>
        <tr>
            <th>DDS Discovery server</th>
            <td align="center">🟨</td>
            <td align="center">✅</td>
            <td align="center">🟨</td>
            <td align="center">✅</td>
            <td align="center">🟨</td>
            <td align="center">❌</td>
            <td align="center">🟠</td>
            <td align="center">❌</td>
        </tr>
        <tr>
            <th>DDS Partitions</th>
            <td align="center">🟨</td>
            <td align="center">✅</td>
            <td align="center">✅</td>
            <td align="center">🟠</td>
            <td align="center">✅</td>
            <td align="center">🟠</td>
            <td align="center">🟨</td>
            <td align="center">❌</td>
        </tr>
    <tbody>
</table>

> ***Legend :***  
> ✅ : Good  
> 🟨 : Fair  
> 🟠 : Poor  
> ❌ : Bad

### Conclusion

Each solution has its advantages and drawbacks, there is no perfect solution. You should select the method that best fits your 
needs, and even **combine** some of them to achieve your goal
(see [this](https://discourse.ros.org/t/restricting-communication-between-robots/2931/32))


First of all, **DDS Discovery servers** can't be used as a standalone solution, as they don't isolate entirely your system and
can cause naming conflicts problems easily.

If **isolation** is not a key factor in your decision, you should probably use the easiest method : **namespacing**.
However, with a large number of robots (and nodes), **network discovery traffic** can become a problem. Adding **discovery
servers** in your architecture would allow to reduce this traffic.

If you need strong isolation, **domain IDs** might be the best strategy for you, as by default everything is accessible only
to your robot, and you decide which topics can be shared. Moreover, it's a DDS parameter that is built into ROS2, that way
it doesn't rely on specific DDS middleware implementations. However, the bridging of topics between nodes require additional
nodes, thus increasing the **computing power** required on the robot.

That way, if **computing power** is a key factor in your decision, you might want to isolate topics using **partitions**,
that don't require any additional nodes. However, since it still broadcasts discovery information, network discovery traffic
could still be an issue with large fleets of robots.


## 5. Comparing the different architectures

Above, we only compared the different technical communication methods using ROS2, without taking into consideration the global 
architecture of the system.

The system can have a :
- **centralized architecture :** there is an entity, that centralizes the information and sends back information to all robots. 
That entity has running nodes and is on the common network between robots.
- **distributed architecture :** there is no central entity, each robot communicates informations to all other robots.

The communication can be :
- **on a common network :** all the robots are on the same network, and communicate through it
- **ad-hoc :** each robot communicates informations to its neighbours (peer to peer)

> The names of these architectures might be different in the litterature

> [!NOTE]
> Any of the previous communication methods could be adapted to work with these architectures

### Pros and cons

A **centralized architecture** has the benefit of being pretty easy to design and implement in the code : each robot sends 
informations (sensors...), and the **central computer** gathers them to send back instructions to the robots.
However, if the central computer fails, all the communication is stopped and this causes the entire fleet to be down.

On the contrary, a **distributed architecture** is much more **resilient** to failure : as **robots are all interconnected**, if 
one fails, the others can still communicate. However, this is harder to design and implement, especially when robots have
to take a decision together. Finally, if the router that the robots are connected to dies, all communication is interrupted.

When having lots of robots on the **same network**, we can experience bandwidth problems, which affects the effectiveness of the
communication. The **ad-hoc communication** can help with these issues, by enabling **peer-to-peer (P2P) communication** between
robots, so that they can **communicate locally** with their neighbours. However, this prevents them from having a global 
organization, but rather local groups that communicate together. Furthermore, this is more costly, as every robot needs to
have the equipment to be able to do such communication. 

### Results

<table>
    <thead>
        <tr style="border:none;">
            <th colspan=2 style="border: none;"></th>
            <th colspan=7 style="text-align:center">Criteria</th>
        </tr>
        <tr>
            <th>Communication</th>
            <th>Architecture</th>
            <!-- Criteria -->
            <th>Resilience</th>
            <th>Scalability</th>
            <th>Organization</th>
            <th>Cost</th>
            <th>Ease of programming</th>
        </tr>
    </thead> 
    <tbody>
        <tr>
            <th rowspan="2">Common network</th>
            <th>Centralized</th>
            <td align="center" rowspan=2>🟠</td>
            <td align="center" rowspan=2>🟠</td>
            <td align="center">✅</td>
            <td align="center" rowspan=2>✅</td>
            <td align="center">✅</td>
        </tr>
        <tr>
            <th>Distributed</th>
            <!-- <td align="center">🟠</td> -->
            <!-- <td align="center">🟠</td> -->
            <td align="center">🟠</td>
            <!-- <td align="center">✅</td> -->
            <td align="center">🟠</td>
        </tr>
        <tr>
            <th rowspan="2">Ad-hoc</th>
            <th>Centralized</th>
            <td align="center">🟨</td>
            <td align="center" rowspan=2>✅</td>
            <td align="center" rowspan=2>🟠</td>
            <td align="center" rowspan=2>🟠</td>
            <td align="center">❌</td>
        </tr>
        <tr>
            <th>Distributed</th>
            <td align="center">✅</td>
            <!-- <td align="center">✅</td> -->
            <!-- <td align="center">🟠</td> -->
            <!-- <td align="center">🟠</td> -->
            <td align="center">🟠</td>
        </tr>
    <tbody>
</table>

> ***Legend :***  
> ✅ : Good  
> 🟨 : Fair  
> 🟠 : Poor  
> ❌ : Bad

> [!NOTE]
> All of these issues (bandwidth, network range...) are pretty hard to simulate

## 6. Quality of service options

Quality of Service Policies allows to further specify the quality of the DDS communication between nodes, in order to reduce
network traffic by not resending data that is not needed when it's lost.

> [!NOTE]
> All of the following options are DDS QoS options but can be configured inside ROS2. However, there are more policies, such
> as `Partition`, `GroupData`... that are only accessible at the DDS level (by changing the default configuration file
> `FASTRTPS_DEFAULT_PROFILES_FILE`)

> See [Documentation](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/getting_started/cpp11/intro_qos.html)

The 2 most important QoS options are
- **Reliability :** Should the arrival of each sample be guaranteed, or the risk of missing a sample acceptable?
  - `BEST_EFFORT` : Do not send data reliably. If samples are lost, they are **not resent**.
  - `RELIABLE` : **Resend** lost samples, depending on the History QoS Policy and Resource Limits QoS Policy.
- **History :** How much data should be stored for reliability and durability purposes?
  - kind : 
    - `KEEP_LAST` : Only keep the last `DEPTH` values to resend
    - `KEEP_ALL` : Keep all failed messages in a queue to resend
  - `DEPTH` : How many samples to keep per instance if Keep Last is specified

### Other policies

When setting the history policy to `KEEP_LAST`, it could overflow the device's memory. The following policies allow to control how much is kept from a device point a view.

- **Resource Limits :** What is the maximum allowed size of a DataWriter’s or DataReader’s queue due to memory constraints?
- **Durability :** Should data be stored and automatically sent to new DataReaders as they start up?
    > *Historical samples = samples that were written before the DataReader was discovered by the DataWriter*
  - `VOLATILE` : Do not save or deliver historical DDS samples.
  - `TRANSIENT_LOCAL` : Save and deliver historical DDS samples if the DataWriter still exists.
  - `TRANSIENT_DURABILITY` : Save and deliver historical DDS samples to store samples in volatile memory.
  - `PERSISTENT_DURABILITY` : Save and deliver historical DDS samples to store samples in non-volatile memory.
- **Deadline :** How do we detect that streaming data is being sent at an acceptable rate?
  - notify the application when a publisher/subscriber sends/receives data within a specific time `PERIOD`

### When to use them ?

- **Streaming data** that does not need reliability at all *(ex: sensor data...)*
- **State data**, where DataReaders generally want to reliably receive the latest state and can accept missing some state updates when the state is changing rapidly
- **Event and Alarm** data that needs guaranteed delivery of every sample

<div align="center"><img src="docs/img/reliability_summary.png" width="850" title="Reliability summary"></div>

<div align="center"><img src="docs/img/qos_summary.png" width="850" title="QoS Summary"></div>

> The durability of an Event/Alarm data depends on your system, whether a late joining subscriber should have access to
> previously published data and whether the data must remain avaiable even if the original publisher is no longer active

You should first design your system depending on the type of data that is communicated, and then add Resource Limits QoS
policies to make sure this configuration works for your system.


## 7. References

Here are the best references, in hierarchical order, on multirobot communication and architecture in ROS2 :

1. [Restricting communication between robots](https://discourse.ros.org/t/restricting-communication-between-robots/2931)
2. [A more unified and standard way of configuring the DDS layer](https://discourse.ros.org/t/a-more-unified-and-standard-way-of-configuring-the-dds-layer/11372/18)
3. [How do you Architect your Robots? State of the Practice and Guidelines for ROS-based Systems](http://acme.able.cs.cmu.edu/pubs/uploads/pdf/ICSE_SEIP_20202020_ICSE_RobotArchitecture.pdf)
    > Robotics architecture standard with ROS (not ROS2) based on real examples
    > ![Guidelines](./docs/img/communication_guidelines.png)




[1]: https://www.researchgate.net/publication/339112401_A_ROS2_based_communication_architecture_for_control_in_collaborative_and_intelligent_automation_systems
