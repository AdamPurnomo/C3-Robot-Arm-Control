#ifndef _PARAMS_H
#define _PARAMS_H

#include "./cfg_arm.h"


/*---------- •¢°º•‡¿©∏ÊÕ—•—•È•·°º•ø --------------------------------*/

extern const double inertia[7];

extern const double Kd[7];
extern const double Kp[7];

extern const double joint_limit[2];
extern const double max_torque[7];




/*---------- µ∞∆ª°¢∞Ã√÷°¶ª—¿™Õ—•—•È•·°º•ø --------------------------*/

extern struct path    path_j[7];
extern struct status  cur_j[7], des_j[7], com_j[7];   /* •∏•Á•§•Û•»∫¬…∏∑œ */
extern struct status  cur_o[7], des_o[7], com_o[7];   /* ¥Ω‡∫¬…∏∑œ */


/*---------- ≥∆¥ÿ¿·§Œ•»•ÅEØ(•«°º•øÕ—) ------------------------------*/

extern double torque[7];

extern int brakeoff_joint;

#endif /* _PARAMS_H */
