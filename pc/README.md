# `pc/` — outils poste de travail

À lancer depuis **ton PC** (ROS 2 Jazzy), le robot tournant de son côté.

| Script | Rôle |
|---|---|
| `./robot_nav.sh` | **RViz navigation** : carte SLAM, scan lidar, plan, robot. Outils *2D Goal Pose* et *2D Pose Estimate*. |
| `./robot_view.sh` | RViz simple : modèle + odométrie fusionnée |
| `./robot_teleop.sh` | pilotage clavier (`teleop_twist_keyboard`) |
| `python3 waypoints.py` | **points de passage EN BOUCLE** : le tableau de points est dans le script. Voir ci-dessous. |

Prérequis : PC et robot sur le **même réseau**, `ROS_DOMAIN_ID=0` des deux côtés
(les scripts s'en chargent).

Joystick navigateur (PC ou téléphone) : `http://<ip-du-robot>:8080/joystick.html`

## Points de passage en boucle

```bash
source /opt/ros/jazzy/setup.bash
python3 waypoints.py
```

**Les points s'éditent dans le script**, en haut du fichier :

```python
POINTS = [
    (1.0, 0.0,    0),
    (1.0, 1.0,   90),
    (0.0, 1.0,  180),
    (0.0, 0.0,  -90),
]
TOURS = 0          # 0 = boucle infinie
```

`(x, y, cap)` : mètres dans le repère `map`, cap en degrés. **L'origine du repère
`map` est l'endroit où le SLAM a démarré**, pas un coin de la pièce.

Le parcours boucle : après le dernier point on repart au premier. `Ctrl+C`
**annule le but en cours**, sinon le robot continuerait de rouler après la fin du
script.

À chaque tour le script affiche les points non atteints et un cumul : c'est ce
qui rend visible une dérive d'odométrie sur une dizaine de tours.

Dépendance à installer une seule fois — deux paquets de **messages**, pas la pile
nav2 (le script le rappelle si elle manque) :

```bash
sudo apt install ros-jazzy-nav2-msgs
```

Prérequis côté robot : nav2 actif (`mowbot status`). C'est son serveur
`waypoint_follower` qui enchaîne les points ; ce script ne fait qu'envoyer le but
et afficher l'avancement. On peut donc le lancer, le tuer et le relancer sans
rien perturber côté robot.
