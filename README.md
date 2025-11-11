# TP1 IROS - Navigation et Localisation Robotique

## 📋 Table des matières

- [Concepts fondamentaux](#concepts-fondamentaux)
- [Topics ROS de la simulation](#topics-ros-de-la-simulation)
- [Contrôle du robot](#contrôle-du-robot)
- [Nœuds du système](#nœuds-du-système)
- [Outils de visualisation](#outils-de-visualisation)
- [Cartographie](#cartographie)

---

## Concepts fondamentaux

### Qu'est-ce que l'odométrie ?

L'**odométrie** est une technique qui utilise les données reçues par des capteurs de position (encodeurs sur les roues, capteurs inertiels, etc.) pour estimer les changements de position du robot au cours du temps. Elle permet de calculer la trajectoire suivie par le robot en intégrant les déplacements successifs.

### Que permet-elle de faire ?

L'odométrie est utilisée sur les robots mobiles pour :

- Estimer le déplacement relatif du robot par rapport à sa position initiale
- Calculer la vitesse instantanée du robot
- Fournir une estimation continue de la pose (position et orientation) du robot
- Servir de base pour d'autres algorithmes de navigation et de localisation

### Est-ce suffisant pour naviguer ?

Non, l'odométrie seule n'est pas suffisante pour une navigation autonome fiable. Voici pourquoi :

- **Accumulation d'erreurs** : Les erreurs de mesure s'accumulent au fil du temps (dérive odométrique)
- **Glissement des roues** : Les roues peuvent glisser, faussant les mesures
- **Référentiel relatif** : L'odométrie donne une position par rapport au point de départ, pas par rapport à l'environnement
- **Pas de correction** : Sans capteurs externes, impossible de corriger les erreurs accumulées

Pour naviguer efficacement, il faut combiner l'odométrie avec d'autres capteurs (LIDAR, caméras) et des algorithmes de localisation comme AMCL.

### Qu'est-ce qu'AMCL ? Que permet-il de réaliser ?

**AMCL** (Adaptive Monte Carlo Localization) est un algorithme de localisation probabiliste qui permet de déterminer la position du robot dans un environnement connu (carte).

**Principe de fonctionnement :**

- Utilise un filtre à particules (Monte Carlo) pour représenter la croyance sur la position du robot
- Compare les données du LIDAR avec la carte de l'environnement
- Chaque particule représente une hypothèse de position possible
- Les particules sont pondérées selon leur cohérence avec les observations
- L'algorithme converge progressivement vers la position réelle

**Avantages :**

- Adaptatif : ajuste le nombre de particules selon l'incertitude
- Robuste aux erreurs d'odométrie
- Permet la re-localisation si le robot se perd

## Topics ROS de la simulation

### Liste des topics disponibles

<details>
<summary>Cliquez pour voir tous les topics</summary>

```
/behavior_server/transition_event
/bond
/bt_navigator/transition_event
/clicked_point
/clock
/cmd_vel                                    ← Commandes de vélocité
/collision_monitor/transition_event
/controller_selector
/controller_server/transition_event
/diagnostics
/docking_server/transition_event
/downsampled_costmap
/downsampled_costmap_updates
/global_costmap/costmap                     ← Carte globale de coûts
/global_costmap/costmap_updates
/global_costmap/global_costmap/transition_event
/global_costmap/voxel_marked_cloud
/goal_checker_selector
/imu                                        ← Centrale inertielle
/initialpose                                ← Position initiale du robot
/joint_states                               ← États des articulations
/local_costmap/costmap                      ← Carte locale de coûts
/local_costmap/costmap_updates
/local_costmap/local_costmap/transition_event
/local_costmap/published_footprint
/local_costmap/voxel_marked_cloud
/local_plan                                 ← Plan de trajectoire local
/map                                        ← Carte de l'environnement
/map_metadata
/map_saver/transition_event
/map_updates
/mobile_base/sensors/bumper_pointcloud
/odom                                       ← Odométrie du robot
/parameter_events
/particle_cloud                             ← Particules AMCL
/plan                                       ← Plan de trajectoire global
/planner_selector
/planner_server/transition_event
/pose                                       ← Pose estimée du robot
/progress_checker_selector
/rgbd_camera/camera_info
/rgbd_camera/depth_image
/rgbd_camera/depth_image/compressed
/rgbd_camera/depth_image/compressedDepth
/rgbd_camera/depth_image/theora
/rgbd_camera/depth_image/zstd
/rgbd_camera/image
/rgbd_camera/image/compressed
/rgbd_camera/image/compressedDepth
/rgbd_camera/image/theora
/rgbd_camera/image/zstd
/robot_description
/rosout
/scan                                       ← Données LIDAR
/slam_toolbox/feedback
/slam_toolbox/graph_visualization
/slam_toolbox/scan_visualization
/slam_toolbox/transition_event
/slam_toolbox/update
/smoother_selector
/smoother_server/transition_event
/tf                                         ← Transformations dynamiques
/tf_static                                  ← Transformations statiques
/velocity_smoother/transition_event
/waypoint_follower/transition_event
/waypoints
```

</details>

### Topics d'intérêt principaux

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Commandes de vélocité pour déplacer le robot |
| `/odom` | `nav_msgs/Odometry` | Odométrie du robot (position estimée) |
| `/scan` | `sensor_msgs/LaserScan` | Données du capteur LIDAR |
| `/map` | `nav_msgs/OccupancyGrid` | Carte de l'environnement |
| `/particle_cloud` | `geometry_msgs/PoseArray` | Particules de l'algorithme AMCL |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Position initiale du robot |
| `/tf` | `tf2_msgs/TFMessage` | Arbre des transformations entre référentiels |

## Contrôle du robot

### Quel topic permet de déplacer le robot ?

Le topic `/cmd_vel` (command velocity) est utilisé pour contrôler le déplacement du robot.

![topic /cmd_vel](/screen/teleop_with_rqt.png)

**Caractéristiques :**

- **Type de message** : `geometry_msgs/Twist`
- **Contenu** : Commandes de vélocité linéaire et angulaire
  - `linear.x` : vitesse avant/arrière (m/s)
  - `linear.y` : vitesse latérale (m/s) - souvent 0 pour robot différentiel
  - `angular.z` : vitesse de rotation (rad/s)

**Exemple de commande :**

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

![terminal teleop avance](/screen/teleop_avance.png)

![terminal teleop avance](/screen/teleop_turn.png)

### Comment fonctionne teleop_twist_keyboard ?

Le package `teleop_twist_keyboard` permet de contrôler le robot avec le clavier.

**Fonctionnement :**

1. Capture les touches du clavier pressées par l'utilisateur
2. Convertit les touches en commandes de vélocité (Twist)
3. Publie ces commandes sur le topic `/cmd_vel`
4. Le contrôleur du robot souscrit à `/cmd_vel` et exécute les commandes

**Mapping des touches typique :**

- `i` : avancer
- `k` : arrêter
- `j` : tourner à gauche
- `l` : tourner à droite
- `u`, `o`, `m`, `,` : mouvements diagonaux

Architecture :

```
Clavier → teleop_twist_keyboard → /cmd_vel → controller_server → Robot
```

---

## Nœuds du système

### 🔧 Nœuds lancés dans ce launch file

Le fichier de lancement démarre plusieurs nœuds organisés par fonctionnalité :

<details>
<summary>Voir la liste complète des nœuds</summary>

```
/behavior_server
/bridge_gz_ros_camera_depth
/bridge_gz_ros_camera_image
/bridge_ros_gz
/bt_navigator
/collision_monitor
/controller_server
/docking_server
/global_costmap/global_costmap
/launch_ros_103001
/lifecycle_manager_navigation
/lifecycle_manager_slam
/local_costmap/local_costmap
/map_saver
/nav2_container
/nav2_rviz_docking_panel_node
/nav2_rviz_selector_node
/planner_server
/robot_state_publisher
/rqt_gui_py_node_92946
/rviz
/rviz_navigation_dialog_action_client
/slam_toolbox
/smoother_server
/transform_listener_impl_57b1dc0d7840
/transform_listener_impl_57b1dc3eb5f0
/transform_listener_impl_5ba2768e07f0
/velocity_smoother
/waypoint_follower
```

</details>

### Classification des nœuds par fonctionnalité

#### Navigation (Nav2)

- **`behavior_server`** : Gère les comportements de navigation (arrêt d'urgence, attente, etc.)
- **`bt_navigator`** : Orchestrateur principal utilisant des arbres de comportements (Behavior Trees)
- **`controller_server`** : Contrôle local du robot (suit la trajectoire)
- **`planner_server`** : Planificateur de trajectoire global
- **`smoother_server`** : Lisse les trajectoires pour des mouvements fluides
- **`velocity_smoother`** : Lisse les commandes de vélocité
- **`waypoint_follower`** : Permet de suivre une séquence de points de passage

#### Cartographie (Costmaps)

- **`global_costmap/global_costmap`** : Carte de coûts globale pour la planification
- **`local_costmap/local_costmap`** : Carte de coûts locale pour l'évitement d'obstacles
- **`collision_monitor`** : Surveillance des collisions en temps réel

#### Localisation et SLAM

- **`slam_toolbox`** : Construction de carte et localisation simultanées (SLAM)
- **`map_saver`** : Sauvegarde des cartes générées

#### État du robot

- **`robot_state_publisher`** : Publie l'état du robot et les transformations TF
- **`transform_listener_impl_*`** : Écoute les transformations entre référentiels

#### Bridges Gazebo-ROS

- **`bridge_gz_ros_camera_depth`** : Pont pour la caméra de profondeur
- **`bridge_gz_ros_camera_image`** : Pont pour l'image caméra
- **`bridge_ros_gz`** : Pont principal Gazebo-ROS

#### Gestion et visualisation

- **`lifecycle_manager_navigation`** : Gestionnaire du cycle de vie des nœuds de navigation
- **`lifecycle_manager_slam`** : Gestionnaire du cycle de vie des nœuds SLAM
- **`rviz`** : Visualisateur 3D
- **`nav2_container`** : Conteneur pour les nœuds Nav2

---

## Outils de visualisation

### Qu'est-ce que RViz ?

**RViz** (ROS Visualization) est un outil de visualisation 3D pour ROS.

**Fonctionnalités principales :**

- **Visualisation en temps réel** : Affiche les données des capteurs (LIDAR, caméras, etc.)
- **Affichage de la carte** : Montre la carte de l'environnement
- **Suivi du robot** : Affiche la position et l'orientation du robot
- **Planification** : Visualise les trajectoires planifiées
- **Interaction** : Permet de définir des objectifs de navigation avec la souris
- **TF Tree** : Visualise l'arbre des transformations entre référentiels

**Utilisation typique :**

- Définir la position initiale du robot (2D Pose Estimate)
- Envoyer des objectifs de navigation (2D Nav Goal)
- Observer les particules AMCL
- Visualiser les données du LIDAR et les costmaps

### Quelle différence avec Gazebo ?

| Critère | Gazebo | RViz |
|---------|--------|------|
| **Type** | Simulateur de physique 3D | Outil de visualisation |
| **Objectif** | Simuler le comportement du robot dans un environnement virtuel | Visualiser les données ROS en temps réel |
| **Physique** | ✅ Simulation complète (gravité, collisions, friction) | ❌ Pas de simulation physique |
| **Capteurs** | ✅ Simule les capteurs (LIDAR, caméras, IMU) | ❌ Affiche seulement les données reçues |
| **Interaction** | ✅ Le robot interagit avec l'environnement | ⚠️ Interaction limitée (définir des objectifs) |
| **Performance** | Plus lourd (calculs physiques) | Plus léger (affichage uniquement) |
| **Utilisation** | Développement et test avant déploiement réel | Débogage et visualisation (simulation ou robot réel) |

**En résumé :**

- **Gazebo** = "Monde virtuel" où le robot évolue
- **RViz** = "Fenêtre" pour observer ce que le robot perçoit

---

## Cartographie

### Carte de l'environnement générée

![Carte générée](/map/IROS.jpeg)

![Carte générée avec a particule](/screen/map_with_particle.png)
### Informations importantes dans les fichiers de carte

Le fichier `IROS.yaml` contient les métadonnées de la carte :

```yaml
image: IROS.pgm                # Fichier image de la carte
mode: trinary                  # Mode d'occupation (3 états)
resolution: 0.050              # Résolution en mètres/pixel
origin: [-3.100, -6.474, 0]   # Origine de la carte (x, y, θ)
negate: 0                      # Inversion des couleurs (0=non)
occupied_thresh: 0.65          # Seuil pour case occupée
free_thresh: 0.196             # Seuil pour case libre
```

#### Explication détaillée des paramètres

**`image`** : Nom du fichier image (format PGM - Portable Gray Map, convertit en jpeg pour l'affichage dans le README)

**`resolution`** : 0.050 m/pixel = **5 cm par pixel**
- Plus la valeur est petite, plus la carte est précise
- Compromis entre précision et taille du fichier

**`origin`** : Point de référence de la carte dans le monde réel
- `[-3.100, -6.474, 0]` : Position (x, y) en mètres et rotation (θ) en radians
- Correspond au coin inférieur gauche de l'image

**`mode: trinary`** : Trois états possibles pour chaque cellule
- **Occupé** (noir) : Obstacle détecté
- **Libre** (blanc) : Espace navigable
- **Inconnu** (gris) : Zone non explorée

**`occupied_thresh`** : 0.65
- Probabilité au-dessus de laquelle une cellule est considérée comme occupée
- Valeur entre 0 et 1

**`free_thresh`** : 0.196
- Probabilité en-dessous de laquelle une cellule est considérée comme libre

**`negate`** : 0
- Si 1 : inverse les couleurs (noir↔blanc)
- Utile selon le format d'export de la carte

### Importance de ces paramètres

Ces paramètres sont cruciaux pour :

- **La navigation** : Le planificateur utilise la résolution pour calculer les trajectoires
- **La localisation** : AMCL compare les scans LIDAR avec cette carte
- **L'évitement d'obstacles** : Les seuils déterminent ce qui est considéré comme obstacle
- **La reproductibilité** : Permet de charger la carte dans différents environnements
 