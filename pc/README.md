# `pc/` — outils poste de travail

À lancer depuis **ton PC** (ROS 2 Jazzy), le robot tournant de son côté.

| Script | Rôle |
|---|---|
| `./robot_nav.sh` | **RViz navigation** : carte SLAM, scan lidar, plan, robot. Outils *2D Goal Pose* et *2D Pose Estimate*. |
| `./robot_view.sh` | RViz simple : modèle + odométrie fusionnée |
| `./robot_teleop.sh` | pilotage clavier (`teleop_twist_keyboard`) |

Prérequis : PC et robot sur le **même réseau**, `ROS_DOMAIN_ID=0` des deux côtés
(les scripts s'en chargent).

Joystick navigateur (PC ou téléphone) : `http://<ip-du-robot>:8080/joystick.html`

## Dépannage

Les topics du robot n'apparaissent pas :
```bash
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
ros2 daemon stop        # repart propre au prochain appel
ros2 topic list
```
Après un changement de WiFi, redémarrer aussi les services côté robot
(`mowbot up`) : les participants DDS restent liés à l'ancienne adresse.

Si la carte n'apparaît pas dans RViz (bug GLSL de certains pilotes), décommenter
`export LIBGL_ALWAYS_SOFTWARE=1` dans `robot_nav.sh`.
