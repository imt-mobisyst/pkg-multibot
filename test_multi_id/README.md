# Test communication multi ID

On utilise la librairie [domain_bridge](https://github.com/ros2/domain_bridge/blob/main/doc/design.md) qui permet de lancer plusieurs noeuds dans un même processus et ainsi pouvoir "bridge" des topics d'un DOMAIN_ID vers un autre.

Pour l'installer : 
```bash
apt install ros-iron-domain-bridge
```

## Test simple avec un talker et listener

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


## Test avec turtle sim

Chaque "robot" sera associé à un domain ID unique *(`bot_domain_id`)*. Dans chaque domain ID, on aura un `turtlesim_node` et un `turtlesim_controller` (qui permettra de déplacer la turtle vers les points).

Lancement des `turtle` dans des terminaux différents (les noeuds sont lancés automatiquement dans le bon ROS_DOMAIN_ID donné en argument du launchfile) :

```bash
ros2 launch test_multi_id turtlesim_bridge_launch.py bot_domain_id:="10" operator_domain_id:="1"
ros2 launch test_multi_id turtlesim_bridge_launch.py bot_domain_id:="11" operator_domain_id:="1"
```


Des noeuds de bridge sont lancés automatiquement par les launchfile pour transmettre les topics nécessaires aux noeuds dans l'`operator_domain_id`. On lancera donc le noeud opérateur (qui s'occupe de gérer la priorité entre les turtle) dans ce domaine :

```bash
ROS_DOMAIN_ID=1 ros2 run test_multi_id operator.py --ros-args -p nb_robots:=2
```


Pour envoyer des points à atteindre, on utilise la commande suivante :
```bash
ROS_DOMAIN_ID=1 ros2 topic pub /target geometry_msgs/msg/PointStamped "{point: {x: 4.5, y: 9.0, z: 0.0}}" --once
```

> Remarque : Au bout d'un certain temps, les noeuds "disparaissaient" : pas de crash, process qui tourne toujours, mais `ros2 node list` et `ros2 topic list` complétement vides. La seule solution qui a semblé marcher est de changer le DDS de **eProsima Fast DDS** vers **Eclipse Cyclone DDS**