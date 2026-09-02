#pragma once
// controllers/mowbot_p4/main/robot.h -- tout ce qui est propre a CE robot.
// Le bloc ci-dessous est l'ancien bloc "#if CONFIG_IDF_TARGET_ESP32P4" de
// main/config.h, deplace A L'IDENTIQUE quand le firmware a ete decoupe en
// controleurs (verifie par diff au deplacement). Seule addition : le nom du
// noeud, qui n'est pas une valeur de calibration.
#define ROBOT_NODE_NAME "mowbot_base"

// ============================================================
//  ROBOT A — Waveshare ESP32-P4-ETH + moteurs 12 V
//  ETAT : CALIBRE ET VALIDE (carres a +/-0.4 cm, coins a +/-1 deg).
//  >>> NE RIEN MODIFIER ICI <<<
// ============================================================
// MDD10A rev 2.0. Seuls les pins du header DROIT sont libres (le gauche est
// reserve SD/camera/C6). GPIO46 MORT, GPIO48 inaccessible.
// M1/M2 etaient inverses vs realite physique -> L = canal M2.
#define PIN_MOTOR_L_PWM   22
#define PIN_MOTOR_L_DIR   23
#define PIN_MOTOR_R_PWM   20
#define PIN_MOTOR_R_DIR   21

#define PIN_ENC_L_A       27     // Bleu
#define PIN_ENC_L_B       47     // Orange
#define PIN_ENC_R_A       33     // Bleu
#define PIN_ENC_R_B       32     // Orange

// I2C de l'IMU sur 3/2 (et non les 7/8 par defaut) pour regrouper le cablage
// sur le meme header que moteurs et encodeurs. Choix VERIFIE :
//  - sur l'ESP32-P4 les broches de strapping sont GPIO34..38, PAS 0..3 comme
//    sur l'ESP32 classique : 2 et 3 sont donc des GPIO ordinaires, sans effet
//    au demarrage ;
//  - elles n'apparaissent dans aucune reservation de la carte (Ethernet
//    31/50/51/52, audio 9..13, ampli 53) ;
//  - aucun conflit avec le projet, contrairement a 47 qui porte l'encodeur
//    gauche et a 48 note inaccessible au bring-up.
// L'I2C est librement placable : le P4 dispose d'une matrice de commutation.
#define PIN_IMU_SDA       3
#define PIN_IMU_SCL       2

// -- geometrie robot 12 V (calibree au sol) --
#define WHEEL_RADIUS_M        0.0753f  // rayon EFFECTIF [m] — RECALIBRE au sol.
                                       // Mesure (nodes/calib_1m.py) : l'odometrie
                                       // annoncait 102.9 cm pour 111 cm reels, soit
                                       // une SOUS-estimation de 7.3 %.
                                       //   0.0698 x 111/102.9 = 0.0753
                                       // Proche du nominal 0.075 (Ø15 cm), ce qui
                                       // conforte la mesure.
                                       // Historique : 0.0698 venait d'une mesure
                                       // inverse (odom 29 cm pour 27 reels, pneu
                                       // ecrase). L'ecart de sens indique un
                                       // changement de roues, de pneus ou de charge
                                       // depuis cette premiere calibration.
#define TRACK_WIDTH_M         0.4607f  // entraxe [m] — RECALIBRE, et confirme par
                                       // TROIS methodes independantes :
                                       //   mesure au metre              : 0.46
                                       //   gyro + encodeurs             : 0.4653
                                       //   rotations reelles au repere  : 0.4607
                                       // Affinage final (nodes/turn360.py) : a 0.465
                                       // le robot depassait de 3 deg sur un tour et de
                                       // 10 deg sur trois, soit ~0.9 % de trop.
                                       //   0.465 x 1080/1090 = 0.4607
                                       // Les deux mesures concordant (0.83 % et
                                       // 0.93 %), l'ecart etait reel et non du bruit
                                       // de lecture -- un seul tour n'aurait pas
                                       // permis de trancher.
                                       // L'ancien 0.59 sur-estimait de 27 % : les
                                       // consignes angulaires etaient donc fausses
                                       // d'autant (le robot tournait trop).
                                       //
                                       // METHODE (nodes/calib_track.py) : comparer la
                                       // rotation du GYRO a celle que les vitesses de
                                       // roue MESUREES impliquent, et non a la
                                       // consigne. Sinon on melange deux causes : ici
                                       // les roues ne font que 83 % de leur consigne,
                                       // ce qui ferait conclure a tort que l'entraxe
                                       // est trop PETIT alors qu'il etait trop GRAND.
#define TICKS_PER_WHEEL_REV   2560.0f  // 64 CPR x4 quadrature x reducteur 10:1

#define MOTOR_L_INVERT    0
#define MOTOR_R_INVERT    0
#define ENC_L_INVERT      0      // suit l'echange L<->R (ex-droit, pins 27/47)
#define ENC_R_INVERT      1      // suit l'echange L<->R (ex-gauche, pins 33/32)

// -- gains PID robot 12 V (regles sur le robot) --
#define MAX_WHEEL_SPEED_MPS_ROBOT  1.0f
#define FF_GAIN  0.0f                       // pas de feed-forward (PID valide tel quel)
#define PID_KP  0.8f
#define PID_KI  2.0f
#define PID_KD  0.0f

