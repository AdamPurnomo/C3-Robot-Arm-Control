/******************************************************
	     Command Functions for PA10
*******************************************************/

#include "main_cfg.h"
#include "include.h"
#include "params_arm.h"


 void allzero(void);

 void allboff(void);
 void allbon(void);

 void boff1(void);
 void boff2(void);
 void boff3(void);
 void boff4(void);
 void boff5(void);
 void boff6(void);
 void boff7(void);

extern void All_OFFBrake(void);
extern void joint_moveto(double *Angle,double Time);
extern void OFFBrake(void);
extern void ONBrake(void);




void
allzero(void)
{
  double angle[7];	
  double desTime;
  int i = 0;
  desTime = 10.;

  for (i = 0; i < 7; i++) angle[i] = 0.;
  joint_moveto(angle, desTime);



  printf("\n\t Joint Control --- allzero!!\n\n");

}

/*============== Break off ===============*/
/*========================================*/

void
allboff(void)
{
  All_OFFBrake();

  printf("\n\tAll axes are brake off!!\n");
}

void
allbon(void)
{
  ONBrake();
  
  printf("\n\tAll axes are brake on!!\n");
}

void
boff1(void)
{
  brakeoff_joint = 0;

  OFFBrake();

  printf("\n\t 1 axis is brake off!!\n");
}

void
boff2(void)
{
  brakeoff_joint = 1;

  OFFBrake();

  printf("\n\t 2 axis is brake off!!\n");
}

void
boff3(void)
{
  brakeoff_joint = 2;

  OFFBrake();

  printf("\n\t 3 axis is brake off!!\n");
}

void
boff4(void)
{
  brakeoff_joint = 3;

  OFFBrake();

  printf("\n\t 4 axis is brake off!!\n");
}

void
boff5(void)
{
  brakeoff_joint = 4;

  OFFBrake();

  printf("\n\t 5 axis is brake off!!\n");
}

void
boff6(void)
{
  brakeoff_joint = 5;

  OFFBrake();

  printf("\n\t 6 axis is brake off!!\n");
}

void
boff7(void)
{
  brakeoff_joint = 6;

  OFFBrake();

  printf("\n\t 7 axis is brake off!!\n");
}


