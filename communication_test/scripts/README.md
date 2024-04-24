# Scripts d'évaluation

Ces scripts permettent de comparer le nombre de paquets envoyés sur le réseau lors du processus de "Discovery" en fonction de l'architecture choisie (Serveurs DDS, multi domain ID, namespaces).

Pour récupérer les données : 

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE="no_intraprocess_configuration.xml"
sudo bash generate_discovery_packages.bash ~/ros2/install/local_setup.bash SERVER
sudo bash generate_discovery_packages.bash ~/ros2/install/local_setup.bash DOMAIN
sudo bash generate_discovery_packages.bash ~/ros2/install/local_setup.bash
```

Pour comparer les données :
```bash
python3 discovery_packets.py
```