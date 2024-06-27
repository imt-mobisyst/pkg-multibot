# Evaluation scriots

> These are based on the [following scripts](https://docs.ros.org/en/foxy/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#compare-fast-dds-discovery-server-with-simple-discovery-protocol)

These scripts allow to compare the number of packets sent to the networ during the "Discovery protocol", depending on the chosen architecture (DDS Discovery servers, Domain ID bridge, namespaces, partitions).

In order to get the data from the network, run the following commands : 

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE="no_intraprocess_configuration.xml"
sudo bash generate_discovery_packages.bash ~/ros2/install/local_setup.bash SERVER
sudo bash generate_discovery_packages.bash ~/ros2/install/local_setup.bash DOMAIN
sudo bash generate_discovery_packages.bash ~/ros2/install/local_setup.bash
```

To compare the data :
```bash
python3 discovery_packets.py
```