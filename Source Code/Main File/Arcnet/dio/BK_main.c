#include "DIO_6464T2.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

int main()
{
  int i;
  BYTE Bk;

  ThreadCtl(_NTO_TCTL_IO,0);

  //ボードの初期化
  if(DioInit() == FALSE) {
    printf("DioInit ERROR!\n");
    return 1;
  }
  else printf("DioInit OK!\n");

  printf("Dio output and input\n");
  printf("Brake ON 5sec\n");
  for(i=0;i<5;i++) {
    //Brake ON
    DioWrite_byte(0x06, 0xFF);

    // /BKを読み出す
    // 0xC0：J1～J6ブレーキオフ, 0xFF：ブレーキオン
    Bk = ~DioRead_byte(0x00);
    printf("BK = %x", Bk);
    printf("\n");

    delay(1000);
  }

  printf("Brake OFF 2sec\n");
  for(i=0;i<2;i++) {
    //Brake ON
    DioWrite_byte(0x06, 0x00);

    // /BKを読み出す
    // 0xC0：J1～J6ブレーキオフ, 0xFF：ブレーキオン
    Bk = ~DioRead_byte(0x00);
    printf("BK = %x", Bk);
    printf("\n");

    delay(1000);
  }

  printf("Brake ON 5sec\n");
  for(i=0;i<5;i++) {
    //Brake ON
    DioWrite_byte(0x06, 0xFF);

    // /BKを読み出す
    // 0xC0：J1～J6ブレーキオフ, 0xFF：ブレーキオン
    Bk = ~DioRead_byte(0x00);
    printf("BK = %x", Bk);
    printf("\n");

    delay(1000);
  }

  DioFin();

  return 0;
}
