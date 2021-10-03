//--------------------------------------------------------------------------------------
// C3をあれするやつ
// ここにはいい感じの説明を入れる
// テキトウ英語を吉田になおしてもらう
// 20181022 yoshinaga
//--------------------------------------------------------------------------------------
#ifndef _DEVICEIF_
#define _DEVICEIF_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "pcidev.h"
#include "../../main_cfg.h"
#include "../../cfg_arm.h"
#include "../pi/PI_128LH.h"
#include "../po/PO_128LH.h"
#include "../dio/DIO_6464T2.h"
#include "../da/DA16_16L.h"
#include "../cnt/CNT32_8M.h"

#ifndef OK
#define OK     (EXIT_SUCCESS)
#endif

#ifndef ERROR
#define ERROR    (EXIT_FAILURE)
#endif


extern const double  max_pos_limit[];
extern const double  min_pos_limit[];
extern const double  hard_max_pos_limit[];
extern const double  hard_min_pos_limit[];

extern const double  vel_limit[];
extern const double  hard_vel_limit[];

extern const double  torq_limit[];
extern const double  hard_torq_limit[];

extern const double  RRatio[];
extern const int     PoOffset[];
extern const double  enc2rad;

//--------------------------------------------------------------------------------------
// You use these functions to control C3;
int   DeviceInit(void);         // Use this function at first for initialize.
int   DeviceStart(void);        // Use this function to start
int   DeviceUpdate(void);       // Use this function to update state.
int   DeviceFin(void);          // Use this function to end of control.
void  D_GetPosition(double*);     // Use this function to gain each joint position.
void  D_SetTorq(double*);         // Use this function to set of each joint torque.
void  D_NoTorq(void);
void  AllBKon(int);

//--------------------------------------------------------------------------------------
// You do not use these functions whitch are used to control servo pacs;
int   GlobalValueInit();
int   BoardInit(void);
int   BoardFin(void);
int   EncAbsPos(void);          // モータの多回転量を取得し，絶対位置を得る．

int   ESTOPcheck(int);
int   SRDYcheck(int);
int   ALMcheck(int);
int   TGONcheck(int);
int   BKcheck(int);

int   CSELout(int);
int   PositionInput(double*,double*);
int   PosToVel(double*, double*, double*, double*);
int   ControlCmdOut(void);
int   LightBulbOn(int);
int   TorqCmdOut(double*);
void  GetCmdVolt(double*);
void  GetPulse(double*);

int   LimitPosVelCheck(double*,double*);
int   LimitPosCheck(double*);
int   LimitVelCheck(double*);
int   HardLimitPosCheck(double*);
int   HardLimitVelCheck(double*);

#endif /* _DEVICEIF_ */




