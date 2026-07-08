#include "odometry.h"
#include "config.h"

#include <math.h>

void odometry_update(odom_state_t *odom, float d_left_m, float d_right_m, float dt)
{
    float d_center = 0.5f * (d_left_m + d_right_m);
    float d_theta  = (d_right_m - d_left_m) / TRACK_WIDTH_M;

    // Intégration au point milieu : exacte au 2e ordre, suffisante à 50 Hz
    odom->x += d_center * cosf(odom->theta + 0.5f * d_theta);
    odom->y += d_center * sinf(odom->theta + 0.5f * d_theta);
    odom->theta += d_theta;

    while (odom->theta > (float)M_PI)  odom->theta -= 2.0f * (float)M_PI;
    while (odom->theta < -(float)M_PI) odom->theta += 2.0f * (float)M_PI;

    odom->v = (dt > 0.0f) ? d_center / dt : 0.0f;
    odom->w = (dt > 0.0f) ? d_theta / dt : 0.0f;
}
