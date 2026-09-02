#pragma once

// Traction ARRIERE sur les DEUX canaux du MDD10A rev 2.0, en sign-magnitude :
// PWM = |vitesse|, DIR = sens. UN MOTEUR PAR ROUE ARRIERE, sans differentiel
// mecanique.
//
// Consequence : les deux roues ne doivent PAS tourner a la meme vitesse en
// virage, sinon elles se battent (ripage, courant, odometrie faussee). C'est
// kin_ackermann.c qui calcule le differentiel ELECTRONIQUE et commande chaque
// roue separement -- ce module ne fait que sortir du PWM.
//
// Cablage IDENTIQUE a mowbot_p4 : meme carte, memes broches, rien a redecabler.

typedef enum { TRACTION_LEFT = 0, TRACTION_RIGHT = 1 } traction_id_t;

void traction_init(void);

// cmd dans [-1.0, 1.0] ; clampe en interne. Negatif = marche arriere.
void traction_set(traction_id_t wheel, float cmd);

// Coupe les DEUX moteurs.
void traction_stop(void);
