#include "./PI_128LH.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

int main()
{
  int i;
  BYTE emgsw;
  BYTE alo;

  ThreadCtl(_NTO_TCTL_IO,0);

  //ボードの初期化
  if(PiInit() == FALSE) {
    printf("PiInit ERROR!\n");
    return 1;
  }
  else printf("PiInit OK!\n");

  printf("Pi output and input\n");
  for(i=0;i<500;i++) {
    alo   = PiRead_byte(0x01);//アラームコード出力
    emgsw = PiRead_byte(0x07);//緊急停止スイッチ
    printf("%d  EMGSW: %x, ALO: %x\n",i,emgsw,alo);

    delay(100);
  }

  PiFin();

  return 0;
}
