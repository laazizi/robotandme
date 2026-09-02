#pragma once

// Moteur de traction UNIQUE sur un canal du MDD10A rev 2.0, en sign-magnitude :
// PWM = |vitesse|, DIR = sens. Repris de motors.c de mowbot, reduit a un canal.

void traction_init(void);

// cmd dans [-1.0, 1.0] ; clampe en interne. Negatif = marche arriere.
void traction_set(float cmd);

void traction_stop(void);
