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
  for(i=0;i<40;i++) {

    printf("Servo:OFF(0x00), Control:Torqu(0x00)\n");
    PoWrite_byte( 0x00, 0x00); //サーボオフ
    PoWrite_byte( 0x01, 0x00); //トルク制御
    delay(1000);

    printf("Servo:ON(0x80),  Control:Speed(0x7F)\n");
    PoWrite_byte( 0x00, 0x7F); //サーボオン
    PoWrite_byte( 0x01, 0x7F); //速度制御
    delay(1000);
  }

  PoFin();

  return 0;
}
