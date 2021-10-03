#ifndef _CFG_ARM_H
#define _CFG_ARM_H

#define GLFW_KEY_W 87
#define GLFW_KEY_S 83
#define GLFW_KEY_D 68
#define GLFW_KEY_A 65
#define GLFW_KEY_Q 81
#define GLFW_KEY_E 69

// �\�t�g�E�F�A�֐ߊp�x���~�b�g [deg]
// ���C�Ȃ̂Ō������ݒ�
#define MAXANG1 (130)
#define MAXANG2 (130)
#define MAXANG3 (160)
#define MAXANG4 (205)
#define MAXANG5 (180)
#define MAXANG6 (115)
#define MAXANG7 (340)

#define MINANG1 (-130)
#define MINANG2 (-30)
#define MINANG3 (-160)
#define MINANG4 (-31)
#define MINANG5 (-180)
#define MINANG6 (-115)
#define MINANG7 (-340)


/* �n�[�h�E�F�A�֐ߊp�x���~�b�g [deg] */
/* �����Ă͂����Ȃ��֐ߊp�x*/
#define HARDMAXANG1 (170 -2)
#define HARDMAXANG2 (50  -2)
#define HARDMAXANG3 (180 -2)
#define HARDMAXANG4 (225 -2)
#define HARDMAXANG5 (200 -2)
#define HARDMAXANG6 (135 -2)
#define HARDMAXANG7 (360 -2)

#define HARDMINANG1 (-170 +2)
#define HARDMINANG2 (-150 +2)
#define HARDMINANG3 (-180 +2)
#define HARDMINANG4 (-51  +2)
#define HARDMINANG5 (-200 +2)
#define HARDMINANG6 (-135 +2)
#define HARDMINANG7 (-360 +2)


/* �n�[�h�E�F�A�֐ߊp�x���~�b�g [deg] */
/* ��΂ɒ����Ă͂����Ȃ��֐ߊp�x*/
// #define HARDMAXANG1 (170)
// #define HARDMAXANG2 (50)
// #define HARDMAXANG3 (180)
// #define HARDMAXANG4 (225)
// #define HARDMAXANG5 (200)
// #define HARDMAXANG6 (135)
// #define HARDMAXANG7 (360)
// 
// #define HARDMINANG1 (-170)
// #define HARDMINANG2 (-150)
// #define HARDMINANG3 (-180)
// #define HARDMINANG4 (-51)
// #define HARDMINANG5 (-200)
// #define HARDMINANG6 (-135)
// #define HARDMINANG7 (-360)

// ���S��̑��x���~�b�g[deg/s]
// ���C�Ȃ̂Ō������ݒ�
#define	MAXSPEED1	(80.)
#define	MAXSPEED2	(80.)
#define MAXSPEED3	(80.)
#define	MAXSPEED4	(80.)
#define	MAXSPEED5	(80.)
#define	MAXSPEED6	(80.)
#define	MAXSPEED7	(80.)

/* �e�}�j�s�����[�^�̍ő哮�쑬�x[deg/s]    */
/* �}�j���A���ɋL�ڂ���Ă���ő哮�쑬�x   */
#define	HARDMAXSPEED1	(450.)
#define	HARDMAXSPEED2	(450.)
#define HARDMAXSPEED3	(431.)
#define	HARDMAXSPEED4	(514.)
#define	HARDMAXSPEED5	(553.)
#define	HARDMAXSPEED6	(553.)
#define	HARDMAXSPEED7	(720.)

/*--- �C�i�[�V�� ---*/
#define INERTIA1   (0.000333)
#define INERTIA2   (0.000152)
#define INERTIA3   (0.00083)
#define INERTIA4   (0.000153)
#define INERTIA5   (0.0000454)
#define INERTIA6   (0.0000573)
#define INERTIA7   (0.0000514)


/*--- �\�t�g�E�F�A�g���N���~�b�g [N�Em] ---*/
#define MAX_TORQUE1   (15.0)
#define MAX_TORQUE2   (30.0)
#define MAX_TORQUE3   (10.0)
#define MAX_TORQUE4   (10.0)
#define MAX_TORQUE5   (5.0)
#define MAX_TORQUE6   (5.0)
#define MAX_TORQUE7   (5.0)

/*--- ���e�ő�g���N�w�ߒl [N�Em] ---*/
#define HARD_MAX_TORQUE1   (1.27)
#define HARD_MAX_TORQUE2   (1.27)
#define HARD_MAX_TORQUE3   (0.477)
#define HARD_MAX_TORQUE4   (0.159)
#define HARD_MAX_TORQUE5   (0.159)
#define HARD_MAX_TORQUE6   (0.159)
#define HARD_MAX_TORQUE7   (0.159)

/*--- Length of arm_link ---*/

// #define LB  (0.0)
// #define LS  (0.450)
// #define LE  (0.480)
// #define LW  (0.070)
// #define HZ  (0.0)



/*--- ���x�Q�C�� ---*/

#define KD1   40
#define KD2   30
#define KD3   50
#define KD4   30
#define KD5   40
#define KD6   47
#define KD7   15


/*--- ���Q�C�� ---*/

#define KP1    (70000.)
#define KP2    (95000.)
#define KP3    (25050.)
#define KP4    (70000.)
#define KP5    (95000.)
#define KP6    (70000.)
#define KP7    (70000.) 

#define SQ2   (1./sqrt(2.))

extern const double  max_pos_limit[7];
extern const double  min_pos_limit[7];
extern const double  vel_limit[7];
extern const int     PoOffset[7];

/*--- parameter for trajectory ---*/

struct status {
  double pos;
  double vel;
  double acc;
};


struct path{
  double pos[6];
  double vel[5];
  double acc[4];
  double time;
};


struct linepath{
  double pos[3][6];
  double vel[3][5];
  double acc[3][4];
  double carvel[3];
  double time;
  double a[3];
  double d[3];
  double T[4][4];
  double pre_pos[7];
  double pre_vel[7];
};


struct circlepath{
  double pos[6];
  double vel[5];
  double acc[4];
  double carvel[3];
  double time;
  double a[3];
  double r;
  double phase;
  double phasevel;
  double T[4][4];
  double init_th[7];
  double pre_pos[7];
  double pre_vel[7];
};

#endif /* _CFG_ARM_H */



