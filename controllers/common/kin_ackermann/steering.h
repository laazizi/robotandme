#pragma once

// Direction par servo RC : impulsion PWM a 50 Hz dont la LARGEUR code l'angle.
// Le servo est asservi en position par construction : aucune boucle a ecrire,
// l'angle EST la commande. Contrepartie : aucun retour, le firmware ne sait
// que ce qu'il a demande (voir config.h, "Servo de direction").

void steering_init(void);

// Angle de braquage en radians, positif a GAUCHE (convention ROS). Clampe a
// +/- STEER_MAX_RAD.
void steering_set(float delta_rad);

// Roues droites. Appele au deadman : une voiture qui perd sa liaison doit
// aller droit, pas garder son dernier braquage.
void steering_center(void);

// Dernier angle COMMANDE (le servo n'a pas de retour).
float steering_get(void);
