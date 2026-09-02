#pragma once
// controllers/ackerbot_p4/main/robot.h -- tout ce qui est propre a CE robot.
// ============================================================
//  ACKERBOT — Waveshare ESP32-P4-ETH presumee — ACKERMANN
//  ETAT : COMPILE, JAMAIS FLASHE NI CABLE. Geometrie en PLACEHOLDERS.
//  Direction par servo RC, traction sur UN canal MDD10A, encodeurs PCNT sur
//  les deux roues arriere. Decisions et raisons : CLAUDE.md, section Ackermann.
// ============================================================
#define ROBOT_NODE_NAME "ackerbot_base"

// ------------------------------------------------------------
// Broches. REPRISES DE MOWBOT_P4, validees sur la meme carte, cablage de CE
// robot non encore verifie. Contraintes de la carte mesurees sur mowbot : seul
// le header DROIT est libre ; GPIO 5, 6, 15, 16 morts ; 46 mort, 48
// inaccessible ; strapping sur 34..38 ; P4 NON tolerant 5 V.
// ------------------------------------------------------------
// Traction : DEUX moteurs arriere, un par roue, sur les DEUX canaux du
// MDD10A -- decision de l'utilisateur du 2 septembre 2026 : on REUTILISE la
// carte et le cablage de mowbot tels quels, on n'ajoute qu'un servo.
// Broches IDENTIQUES a mowbot_p4, validees au banc : rien a redecabler.
// Pas de differentiel mecanique => differentiel ELECTRONIQUE obligatoire,
// calcule dans kin_ackermann.c (roue interieure ralentie en virage).
#define PIN_MOTOR_L_PWM    22
#define PIN_MOTOR_L_DIR    23
#define PIN_MOTOR_R_PWM    20
#define PIN_MOTOR_R_DIR    21
// Direction : servo RC, PWM 50 Hz. C'est le SEUL signal ajoute par ce robot,
// donc la seule broche qui n'est pas heritee de mowbot.
//
// GPIO 14 : VALIDEE AU BANC le 3 septembre 2026 -- le servo balaie.
// C'est un resultat CONTRE-INTUITIF qu'il faut garder : ses voisines
// immediates 5, 6, 15 et 16 sont mortes (header gauche, prises en interne par
// la SD, la camera MIPI et l'ESP32-C6), et j'en avais deduit que 14 le serait
// aussi. C'est faux : 14 fonctionne. La liste des broches mortes de cette
// carte est donc a prendre au pied de la lettre, sans extrapoler aux voisines.
// Repli si besoin un jour : GPIO 45, header droit, jamais essayee.
//
// Le servo s'alimente SEPAREMENT en 5-6 V, jamais depuis le P4 ; son signal
// 3,3 V est accepte par la plupart des servos RC.
//
#define PIN_SERVO          14
// Encodeurs des roues ARRIERE. On GARDE les noms L/R : encoders.c est commun
// et n'a pas a savoir qu'il s'agit d'un essieu arriere.
#define PIN_ENC_L_A        27
#define PIN_ENC_L_B        47
#define PIN_ENC_R_A        33
#define PIN_ENC_R_B        32
#define PIN_IMU_SDA        3
#define PIN_IMU_SCL        2

// ------------------------------------------------------------
// Geometrie. PLACEHOLDERS d'un chassis type RC 1/10 : ils permettent de
// COMPILER, pas de naviguer. Procedure :
//   1. STEER_X_M et STEER_MAX_RAD au metre et au rapporteur, roue en butee ;
//   2. TICKS_PER_WHEEL_REV en tournant une roue d'un tour (tick_count.py) ;
//   3. WHEEL_RADIUS_M sur 1 m au sol (calib_1m.py).
// ------------------------------------------------------------
// GEOMETRIE IDENTIQUE A ROBOT A (utilisateur, 2 septembre 2026) : meme chassis,
// memes roues, memes moteurs. Seule la direction change : les roues folles
// laissent place a UNE roue directrice commandee par servo. Les trois valeurs
// ci-dessous sont donc CALIBREES AU SOL, pas des placeholders -- voir
// mowbot_p4/main/robot.h pour la methode et l'historique de chaque mesure.
#define TRACK_WIDTH_M        0.4607f  // entraxe des roues motrices [m] : mesure
                                      // au metre, gyro+encodeurs et rotations
                                      // reelles concordent (3 methodes).
#define WHEEL_RADIUS_M       0.0753f  // rayon EFFECTIF [m], recalibre au sol.
#define TICKS_PER_WHEEL_REV  2560.0f  // 64 CPR x4 quadrature x reducteur 10:1

// LES DEUX SEULES INCONNUES QUI RESTENT. Elles ne peuvent PAS etre heritees :
// un diffdrive n'a ni empattement ni braquage, ces deux cotes n'existent nulle
// part dans robot A et n'ont jamais ete mesurees.
// POSITION SIGNEE de la roue directrice sur l'axe x, depuis l'essieu moteur.
// +x = AVANT : convention ROS, confirmee par front_marker a +0.205 dans
// robot/config/mowbot.urdf (dont le commentaire d'en-tete, lui, est faux).
//   > 0  roue DEVANT   : tricycle classique,      w = +v tan(delta)/|x|
//   < 0  roue DERRIERE : direction arriere,        w = -v tan(delta)/|x|
// Le signe n'est PAS un detail : a braquage egal le robot tourne du cote
// OPPOSE. Verifie par les contraintes de non-glissement (test/).
//
// MESURE (utilisateur, 3 septembre 2026) : la roue est 20 cm derriere le
// lidar, cote exterieur, et le lidar est a -0.16 m de l'essieu moteur (mesure
// au metre, MOWBOT_LIDAR_X de robot/bin/run_tf.sh et l'URDF concordent).
//   -0.16 - 0.20 = -0.36  ->  roue directrice A L'ARRIERE.
// (une premiere annonce donnait 15 cm, soit -0.31 : corrige le 3 septembre.)
#define STEER_X_M            (-0.36f)
// 45 deg = 0,7854 rad. DEDUIT, a confirmer au banc : servo du marche 180 deg,
// qui parcourt ces 180 deg sur 500-2500 us ; or SERVO_MIN_US/MAX_US valent
// 1000/2000, soit la MOITIE de la plage -> 90 deg de course, +-45 deg.
// C'est la CONFIGURATION qui limite, pas la mecanique : passer a 500/2500
// donnerait +-90 deg et rendrait la rotation sur place possible. Choix de
// l'utilisateur du 2 septembre 2026 : rester a +-45 deg, mode tricycle.
// A RETENIR : la valeur finale est le MINIMUM entre la course du servo et la
// butee mecanique de la timonerie. Mesurer au rapporteur, roue en butee.
// Consequence a 45 deg : roue exterieure a 164 % de la vitesse d'essieu (voir
// ackermann_wheel_targets), rayon de braquage minimal 0,360 m. La roue
// interieure ne s'inverse pas : il faudrait 57,4 deg.
#define STEER_MAX_RAD        0.7854f

// Le rayon de braquage minimal n'est PAS defini ici : c'est
// ackermann_min_turning_radius() (ackermann.h) qui le derive de STEER_X_M et
// STEER_MAX_RAD. Il y avait ici une macro qui codait tan(0,52) en dur et ne
// suivait donc pas STEER_MAX_RAD -- 75 % d'erreur a 45 deg.

// ------------------------------------------------------------
// Servo. PLACEHOLDERS STANDARD RC. UNE SEULE roue directrice : le modele
// bicyclette est donc EXACT et non approche, delta est l'angle physique de la
// roue -- pas l'angle d'une roue virtuelle comme sur un Ackermann a deux roues
// directrices avec trapeze de direction. Le servo N'A PAS DE RETOUR : l'angle que
// l'odometrie utilise est l'angle COMMANDE -- premiere source d'erreur de
// cette odometrie. Un potentiometre sur la timonerie leverait la limite.
// ------------------------------------------------------------
#define SERVO_FREQ_HZ        50
#define SERVO_CENTER_US      1500
#define SERVO_MIN_US         1000    // butee ELECTRIQUE du servo, pas la mecanique
#define SERVO_MAX_US         2000
#define SERVO_US_PER_RAD     (500.0f / STEER_MAX_RAD)
#define SERVO_INVERT         0       // 1 si braquer a gauche tourne les roues a droite

// ------------------------------------------------------------
// Traction et regulation. Gains de mowbot 12 V, A REGLER ICI.
// ------------------------------------------------------------
#define MAX_SPEED_MPS        1.0f
// Inversions REPRISES DE MOWBOT_P4 : memes moteurs, memes encodeurs, memes
// broches, meme cablage. ENC_R_INVERT=1 y est le resultat d'une calibration au
// sol (l'echange L<->R des canaux M1/M2, cf. memoire board-usable-gpio).
// A reverifier quand meme au banc : si le chassis Ackermann inverse un moteur
// ou un encodeur par rapport a la tondeuse, c'est ici que ca se corrige.
#define MOTOR_L_INVERT       0
#define MOTOR_R_INVERT       0
#define ENC_L_INVERT         0       // au banc : chaque roue doit compter + en avant
#define ENC_R_INVERT         1
#define FF_GAIN              0.0f    // mowbot suivait a 83 % sans FF : 1.0 est un bon depart
#define PID_KP               0.8f
#define PID_KI               2.0f
#define PID_KD               0.0f

// Sous cette vitesse, nav2 demande une rotation SUR PLACE, impossible en
// Ackermann : on braque dans le sens demande sans avancer (ackermann.c).
#define V_EPS_MPS            0.02f
#define W_EPS_RADPS          0.02f
