#ifndef _MAIN_CFG_H
#define _MAIN_CFG_H

/*-------------------- Timer --------------------*/
#ifndef FREQ
#define FREQ       (1000)    /* Hz */
#endif

#ifndef TICKS
#define TICKS      (1./FREQ)  /* sec */
#endif

#ifndef nTICKS
#define nTICKS (1000000000./FREQ)
#endif

/*-------------------- Definition --------------------*/
#ifndef TRUE
#define TRUE   (1)
#endif

#ifndef FALSE
#define FALSE  (0)
#endif

#ifndef ON
#define ON     (1)
#endif

#ifndef OFF
#define OFF    (0)
#endif

#ifndef PI
#define PI     (3.14159265358979323846)
#endif

#ifndef DEG2RAD
#define DEG2RAD    (PI/180.0)
#endif

#ifndef Deg2Rad
#define Deg2Rad    (PI/180.0)
#endif

#ifndef RAD2DEG
#define RAD2DEG    (180.0/PI)
#endif

#ifndef Rad2Deg
#define Rad2Deg    (180.0/PI)
#endif

/*----- ÉÇÅ[Éh -----*/

enum modes{circle_mode,line_mode,joint_mode,allbrakeoff_mode,brakeoff_mode,nop};

enum exe_modes{operation, simulation}CtrlMode;

struct params {
  int trig;
  int mode;
  int joint;
  double desPos[7];
  double desTime;
};

#endif /* _MAIN_CFG_H */

