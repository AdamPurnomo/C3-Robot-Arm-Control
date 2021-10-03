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
  for(i=0;i<18;i++) {

    //ch9に3V出力
    DaOut_volt(9,-9 + i);
    printf("ch9: %d[V]\n",-9+i);
    delay(1500);

  }

  DaFin();

  return 0;
}
