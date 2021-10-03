#include "DeviceIF.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

int main()
{
  int i,j;
  double pos[7];
  double torq[7];
  double volt[7];

  ThreadCtl(_NTO_TCTL_IO,0);
  CtrlMode = operation;

  //ƒ{[ƒh‚Ì‰Šú‰»
  if( DeviceInit() == OK )  printf("init ok\n");
  else                      printf("init error\n");

  //for(i=0;i<7;i++) torq[i] = torq_limit[i]*(i+1)/7;
  for(i=0;i<7;i++) torq[i] = 0;
  torq[6] = torq_limit[6]*0.1;
  printf("Set torque:");
  for(j=0;j<7;j++) printf("%f, ",torq[j]);


  printf("DeviceUpdate\n");
  for(i=0;i<30;i++) {

    printf("%d[s]:",i*1);
    if( DeviceUpdate() == ERROR ) printf("update error\n");

    D_GetPosition(pos);
    for(j=0;j<7;j++)
      printf("%f, ",pos[j]*RAD2DEG);
    printf("\n");

    D_SetTorq(torq);
    GetCmdVolt(volt);

    printf("raw volt: ");
    for(i=0;i<7;i++) printf(" %f",volt[i]);
    printf("\n");

    printf("output ratio: ");
    for(i=0;i<7;i++) printf(" %f",volt[i]/3.);
    printf("\n");

    delay(1000);
  }

  printf("DeviceFin\n");
  DeviceFin();

  return 0;
}
