#include "DA16_16L.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

int main()
{
  int i;

  ThreadCtl(_NTO_TCTL_IO,0);

  //ボードの初期化
  if(DaInit() == FALSE) {
    printf("DaInit ERROR!\n");
    return 1;
  }
  else printf("DaInit OK!\n");

  printf("DA output\n");
  for(i=0;i<20;i++) {

    //ch9に3V出力
    DaOut_volt(0,0.35);
    printf("ch0: %f[V]\n",0.5);
    delay(1000);

  }

  DaFin();

  return 0;
}
