#include "DeviceIF.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

int main()
{
  int i;
  double torq[7];
  double volt[7];

  for(i=0;i<7;i++) torq[i] = torq_limit[i]*(i+1)/7;

  TorqCmdOut(torq);
  GetCmdVolt(volt);

  printf("torq: ");
  for(i=0;i<7;i++) printf(" %f",torq[i]);
  printf("\n");

  TorqCmdOut(torq);
  GetCmdVolt(volt);

  printf("volt: ");
  for(i=0;i<7;i++) printf(" %f",volt[i]);
  printf("\n");

  return 0;
}
