#pragma once
// Faux config.h pour le TEST HOTE de la cinematique Ackermann. Il remplace
// controllers/common/base/config.h, qui inclut robot.h et tout l'ESP-IDF.
// Chaque valeur est surchargeable par -D pour tester plusieurs geometries
// (roue directrice devant / derriere, notamment).

#ifndef STEER_X_M
#define STEER_X_M       (+0.36f)   // ackerbot_p4 : roue directrice A L AVANT
#endif
#ifndef TRACK_WIDTH_M
#define TRACK_WIDTH_M    0.4607f   // entraxe calibre de robot A
#endif
#ifndef STEER_MAX_RAD
#define STEER_MAX_RAD    0.7854f    // 45 deg : ackerbot_p4
#endif
#ifndef MAX_SPEED_MPS
#define MAX_SPEED_MPS    1.0f
#endif
#ifndef V_EPS_MPS
#define V_EPS_MPS        0.02f
#endif
#ifndef W_EPS_RADPS
#define W_EPS_RADPS      0.02f
#endif
