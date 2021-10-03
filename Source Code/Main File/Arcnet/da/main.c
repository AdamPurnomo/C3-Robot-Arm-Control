#include "DA16_16L.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

int main()
{
  int i;
  double  volt[7];

  for(i=0;i<7;i++) volt[i] = 0;

  ThreadCtl(_NTO_TCTL_IO,0);

  //ƒ{[ƒh‚Ì‰Šú‰»
  if(DaInit() == FALSE) {
    printf("DaInit ERROR!\n");
    return 1;
  }
  else printf("DaInit OK!\n");

  printf("DA output\n");
  for(i=0;i<20;i++) {
    printf("volt output!\n");
    DaOut_c3kai(volt);
    delay(1000);

  }

  return 0;
}
