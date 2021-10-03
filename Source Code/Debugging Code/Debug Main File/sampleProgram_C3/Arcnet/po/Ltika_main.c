#include "./PO_128LH.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

int main()
{
  int i;
  //BYTE Bk;
  printf("debug1");
  ThreadCtl(_NTO_TCTL_IO,0);
  printf("debug2");

  //ボードの初期化
  if(PoInit() == FALSE) {
	  printf("debug3");

    printf("PoInit ERROR!\n");
    return 1;
  }else printf("PoInit OK!\n");

  printf("Po output and input\n");
  for(i=0;i<8;i++) {
    printf("L tika: %d\n",i);

    PoWrite_byte( 0x02, 0x01); //C3の電球をつける
    delay(500);

    PoWrite_byte( 0x02, 0x00); //C3の電球を消す
    delay(500);
  }

  PoFin();

  return 0;
}
