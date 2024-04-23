# Test communication entre plusieurs robots

## Communication multi DOMAIN_ID

On utilise la librairie [domain_bridge](https://github.com/ros2/domain_bridge/blob/main/doc/design.md) qui permet de lancer plusieurs noeuds dans un même processus et ainsi pouvoir "bridge" des topics d'un DOMAIN_ID vers un autre.

Pour l'installer : 
```bash
apt install ros-iron-domain-bridge
```

### Test simple avec un talker et listener

Il suffit de créer un fichier de configuration qui indique quels topics transmettre, de quel DOMAIN_ID et vers quel DOMAIN_ID *(cf [talker_bridge_config.yaml](./config/talker_bridge_config.yaml))*

Pour lancer le bridge, on utilise la commande suivante :

```bash
ros2 run domain_bridge domain_bridge <path_to>/bridge_config.yaml
```

> On peut lancer cette commande dans un terminal dont le DOMAIN_ID est quelconque

Dans un autre terminal, on lance le noeud `talker` :

```bash
ROS_DOMAIN_ID=2 ros2 run demo_nodes_py talker
```

Dans un autre terminal, on lance le noeud `listener` :

```bash
ROS_DOMAIN_ID=3 ros2 run demo_nodes_py listener
```


### Test avec turtle sim

Chaque "robot" sera associé à un domain ID unique *(`bot_domain_id`)*. Dans chaque domain ID, on aura un `turtlesim_node` et un `turtlesim_controller` (qui permettra de déplacer la turtle vers les points).

Il suffira de lancer le launchfile suivant qui s'occupera de tout lancer (`turtle`, `operator` et `rviz`) :
```bash
ros2 launch communication_test turtlesim_bridge_launch.py
```
---

Si vous souhaitez cependant lancer tout cela à la main, voici la les commandes :

Lancement des `turtle` dans des terminaux différents (les noeuds sont lancés automatiquement dans le bon ROS_DOMAIN_ID donné en argument du launchfile) :

```bash
ros2 launch communication_test turtlesim_robot__launch.py bot_domain_id:="10" operator_domain_id:="1"
ros2 launch communication_test turtlesim_bridge_robot_launch.py bot_domain_id:="11" operator_domain_id:="1"
```


Des noeuds de bridge sont lancés automatiquement par les launchfile pour transmettre les topics nécessaires aux noeuds dans l'`operator_domain_id`. On lancera donc le noeud opérateur (qui s'occupe de gérer la priorité entre les turtle) dans ce domaine :

```bash
ROS_DOMAIN_ID=1 ros2 run communication_test operator.py --ros-args -p nb_robots:=2
```

---

Pour envoyer des points à atteindre, on utilise la commande suivante :
```bash
ROS_DOMAIN_ID=1 ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{pose: {position: {x: 9, y: 9.0, z: 0.0}}}" --once
```

> Remarque : Au bout d'un certain temps, les noeuds "disparaissaient" : pas de crash, process qui tourne toujours, mais `ros2 node list` et `ros2 topic list` complétement vides. La seule solution qui a semblé marcher est de changer le DDS de **eProsima Fast DDS** vers **Eclipse Cyclone DDS**


## Partitionnement des réseaux avec FastDDS server

DDS est le protocole utilisé par ROS pour la communication entre les noeuds. Une partie de ce protocole consiste à chercher sur le réseau avec quels éléments un noeud est capable de communiquer, c'est le "discovery protocol".

Dans notre cas, on utilise un [Discovery server](https://docs.ros.org/en/iron/Tutorials/Advanced/Discovery-Server/Discovery-Server.html) du protocole Fast DDS, afin de simuler des "routeurs" et ainsi isoler des sous réseaux DDS. Exemple :

![Virtual discovery partitions](https://docs.ros.org/en/iron/_images/ds_partition_example.svg "Exemple de partitionnement DDS")

### Test simple avec un talker et listener

On lancera plusieurs serveurs DDS : 
- un pour isoler un réseau "local", dont le port est `11811`. Celui-ci représentera celui qui serait sur l'un des PC d'un robot.
- un pour le réseau commun à tous les robots, dont le port est `11812`. Celui-ci représentera celui qui serait sur le PC opérateur.

```bash
fastdds discovery -i 0 -l 127.0.0.1 -p 11811 # Local
fastdds discovery -i 1 -l 127.0.0.1 -p 11812 # Commun
```

---

**Test 1 :** On vérifie qu'un `talker` en local peut être écouté par un noeud en local et également un noeud en commun MAIS pas par un noeud de l'opérateur
```bash
# Simule un noeud local sur le robot
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_py talker
```
```bash
# Simule un noeud local sur le robot
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_py listener
```
```bash
# Simule un noeud subnet sur le robot
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11812"
ros2 run demo_nodes_py listener
```
```bash
# Simule un noeud de l'opérateur
export ROS_DISCOVERY_SERVER=";127.0.0.1:11812"
ros2 run demo_nodes_py listener
```

Les 2 premiers `listeners` devraient recevoir les messages publiés, mais pas le noeud "opérateur.

---

**Test 2 :** On vérifie qu'un `talker` dans le réseau commun peut être écouté par un noeud en local et également un noeud en commun
```bash
# Simule un noeud subnet sur le robot
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11812"
ros2 run demo_nodes_py talker
```
```bash
# Simule un noeud local sur le robot
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_py listener
```
```bash
# Simule un autre noeud subnet sur le robot
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11812"
ros2 run demo_nodes_py listener
```
```bash
# Simule un noeud de l'opérateur
export ROS_DISCOVERY_SERVER=";127.0.0.1:11812"
ros2 run demo_nodes_py listener
```

Les 3 listeners devraient recevoir les messages publiés.

---

> **Remarque :** Par défaut, le ROS2 CLI créé un noeud pour découvrir le reste du réseau de noeuds. Pour que cela fonctionne, il faut que le ROS2 soit configuré comme **"Super client"**.
Cela se fait avec la commande suivante :
> ```bash
> export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/super_client_configuration_file.xml
> ```
> Attention cependant, vous aurez ainsi accès à tous les noeuds du graphe, **sans aucune partition du réseau**.
