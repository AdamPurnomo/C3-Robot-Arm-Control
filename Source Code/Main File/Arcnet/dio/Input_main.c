#include "DIO_6464T2.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

int main()
{
  int i;
  BYTE  bk;
  BYTE  srdy;
  BYTE  alm;
  BYTE  tgon;

  ThreadCtl(_NTO_TCTL_IO,0);

  //ボードの初期化
  if(DioInit() == FALSE) {
    printf("DioInit ERROR!\n");
    return 1;
  }
  else printf("DioInit OK!\n");

  printf("Dio output and input\n");
  for(i=0;i<500;i++) {
    //DioWrite_byte(0x06, 0x00);				// SELを非選択状態にする

    // /BKを読み出す
    // 0xC0：J1～J6ブレーキオフ, 0xFF：ブレーキオン
    bk = ~DioRead_byte(0x03);

    // /S-RDYを読み出す
    // S-RDY : サーボパックがサーボオン(/S-ON)信号を受付可能かどうか読む
    // 0xC0：サーボレディ(6軸)，0x80：サーボレディ(7軸)
    srdy = ~DioRead_byte(0x00);

    // ALMを読み出す
    // ALM : 異常を検出したときに，オフ(開)にする
    // 0xFE：異常なし 0xFF：アラームあり
    alm = ~DioRead_byte(0x01);

    // TGON : サーボモータが設定値以上の回転速度になった場合オン(閉)する
    // 0xFF：J1～J6停止中、0xC0：J1～J6回転中
    tgon = ~DioRead_byte(0x02);

    printf("%d  /BK: %x, /SRDY: %x, /ALM: %x, /TGON: %x\n",i,bk,srdy,alm,tgon);

    delay(100);
  }

  DioFin();

  return 0;
}
