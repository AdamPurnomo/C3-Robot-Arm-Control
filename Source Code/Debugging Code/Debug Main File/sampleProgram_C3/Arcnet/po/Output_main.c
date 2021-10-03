#include "./PO_128LH.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

int main()
{
  int i;

  ThreadCtl(_NTO_TCTL_IO,0);

  //ボードの初期化
  if(PoInit() == FALSE) {
    printf("PoInit ERROR!\n");
    return 1;
  }else printf("PoInit OK!\n");

  printf("Po output and input\n");
  for(i=0;i<4;i++) {

    PoWrite_byte( 0x00, 0x00); //サーボオフ
    PoWrite_byte( 0x01, 0x00); //トルク制御
    delay(1000);

    PoWrite_byte( 0x00, 0x80); //サーボオン
    PoWrite_byte( 0x01, 0x7F); //速度制御
    delay(1000);
  }

  PoFin();

  return 0;
}
