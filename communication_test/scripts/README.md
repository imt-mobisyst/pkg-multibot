# Evaluation scripts

> These are based on the [following scripts](https://docs.ros.org/en/foxy/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#compare-fast-dds-discovery-server-with-simple-discovery-protocol)

These scripts allow to compare the number of packets sent to the network during the "Discovery protocol", depending on the chosen architecture (DDS Discovery servers, Domain ID bridge, namespaces, partitions, Zenoh).

## Requirements

- [Wireshark](https://www.wireshark.org/) : `sudo apt install wireshark-common`
- [Zenoh plugin for Wireshark](https://github.com/ZettaScaleLabs/zenoh-dissector)

## Usage

In order to get the data from the network, run the following commands : 

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE="no_intraprocess_configuration.xml"
sudo bash generate_discovery_packages.bash <ROS_WS_SETUP_PATH> SERVER
sudo bash generate_discovery_packages.bash <ROS_WS_SETUP_PATH> DOMAIN
sudo bash generate_discovery_packages.bash <ROS_WS_SETUP_PATH> PARTITION
sudo bash generate_discovery_packages.bash <ROS_WS_SETUP_PATH> ZENOH
sudo bash generate_discovery_packages.bash <ROS_WS_SETUP_PATH>
```
> This will launch the operator nodes and start listening for network traffic

For each of them, start the corresponding launchfiles on the robots as soon as you run the command.

Then, to generate graphs comparing the previously generated data, run the following command:
```bash
python3 discovery_packets.py
```