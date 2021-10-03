#include "DeviceIF.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

int main()
{
  int i,j;
  double pos[7];
  double torq[7];

  ThreadCtl(_NTO_TCTL_IO,0);
  CtrlMode = operation;

  //ƒ{[ƒh‚Ì‰Šú‰»
  if( DeviceInit() == OK )  printf("init ok\n");
  else                      printf("init error\n");

  for(i=0;i<7;i++) torq[i] = 0;
  torq[0] = 0.01;

  printf("DeviceUpdate\n");
  for(i=0;i<30;i++) {

    printf("%d[s]:",i*1);
    if( DeviceUpdate() == ERROR ) printf("update error\n");

    GetPosition(pos);
    for(j=0;j<7;j++)
      printf("%f, ",pos[j]*RAD2DEG);
    printf("\n");

    SetTorq(torq);
    printf("Set torque:");
    for(j=0;j<7;j++)
      printf("%f, ",torq[j]);
    printf("\n");

    delay(1000);
  }

  printf("DeviceFin\n");
  DeviceFin();

  return 0;
}
