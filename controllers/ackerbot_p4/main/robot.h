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
// GPIO 45 est un CANDIDAT, PAS UNE BROCHE VALIDEE. Raisonnement : il n'est
// utilise par aucun controleur, il est sur le meme header que GPIO 47 (utilise
// par l'encodeur gauche), et il n'a jamais ete signale mort -- contrairement a
// 46 (muet au banc), 48 (inaccessible) et 5/6/15/16 (header gauche, morts).
// A CONFIRMER au banc : kin_bench_test() balaye le servo au boot, il suffit de
// regarder s'il bouge. S'il reste inerte, essayer une autre broche du header
// DROIT et corriger ici. Le brochage Waveshare n'est pas une preuve : le
// header gauche est etiquete "GPIO" et s'est revele mort sur cette carte.
//
// Le servo s'alimente SEPAREMENT en 5-6 V, jamais depuis le P4 ; son signal
// 3,3 V est accepte par la plupart des servos RC.
#define PIN_SERVO          45
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
//   1. WHEELBASE_M et STEER_MAX_RAD au metre et au rapporteur, roues en butee ;
//   2. TICKS_PER_WHEEL_REV en tournant une roue d'un tour (tick_count.py) ;
//   3. WHEEL_RADIUS_M sur 1 m au sol (calib_1m.py).
// ------------------------------------------------------------
#define WHEELBASE_M          0.30f   // L : axe arriere -> axe avant [m]
#define TRACK_WIDTH_M        0.25f   // entre les roues arriere [m]
#define WHEEL_RADIUS_M       0.05f   // rayon EFFECTIF des roues arriere [m]
                                     // SI les roues de la tondeuse sont
                                     // reutilisees : 0.0753f, mesure au sol
                                     // (cf. mowbot_p4/main/robot.h).
#define TICKS_PER_WHEEL_REV  2560.0f
#define STEER_MAX_RAD        0.52f   // 30 deg, en BUTEE MECANIQUE

// Rayon de braquage minimal, DERIVE : R = L / tan(delta_max). C'est la valeur a
// fournir a nav2 (SmacPlannerHybrid minimum_turning_radius, MPPI min_turning_r).
// Ne pas la saisir a la main cote SBC sans la recalculer d'ici.
#define MIN_TURNING_RADIUS_M (WHEELBASE_M / 0.5726f)   // tan(0.52) = 0.5726

// ------------------------------------------------------------
// Servo. PLACEHOLDERS STANDARD RC. Le servo N'A PAS DE RETOUR : l'angle que
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
