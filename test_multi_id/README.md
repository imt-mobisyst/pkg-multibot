# Test communication multi ID

On utilise la librairie [domain_bridge](https://github.com/ros2/domain_bridge/blob/main/doc/design.md) qui permet de lancer plusieurs noeuds dans un même processus et ainsi pouvoir "bridge" des topics d'un DOMAIN_ID vers un autre.

## Test simple avec un talker et listener

Il suffit de créer un fichier de configuration qui indique quels topics transmettre, de quel DOMAIN_ID et vers quel DOMAIN_ID *(cf [bridge_config.yaml](./config/bridge_config.yaml))*

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