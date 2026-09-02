#pragma once

#include <stdint.h>

// Lecture encodeurs quadrature via le périphérique PCNT :
// comptage 100% matériel (décodage x4), aucune interruption GPIO,
// aucun tick perdu quelle que soit la vitesse.

typedef enum {
    ENCODER_LEFT = 0,
    ENCODER_RIGHT = 1,
} encoder_id_t;

void encoders_init(void);

// Compteur cumulé signé (les débordements ±32000 sont accumulés en interne).
int64_t encoder_get_ticks(encoder_id_t encoder);
