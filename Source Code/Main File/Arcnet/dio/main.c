#include "DIO_6464T2.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

void BKon(int joint, int is_bk)
{
  BYTE bk;

  if (2 < joint && joint < 7) joint--;
  else if (joint == 2)        joint = 6;

  // dioボードのアウトプット信号(o-80からo-87)の状態を読む
  bk = DioRead_byte(0x08+0x00);

  if(is_bk) bk |=   1 << joint;
  else      bk &= ~(1 << joint);

  DioWrite_byte( 0x00, bk);
}

void AllBKon(int is_bk)
{
  if(is_bk) DioWrite_byte(0x00, 0xFF);
  else      DioWrite_byte(0x00, 0x00);
}

int main()
{
  int i;

  ThreadCtl(_NTO_TCTL_IO,0);

  //ボードの初期化
  if(DioInit() == FALSE) {
    printf("DioInit ERROR!\n");
    return 1;
  }
  else printf("DioInit OK!\n");

  printf("BKtest\n");

  printf("Brake OFF joint: ALL\n");
  for(i=0;i<2;i++) {
    AllBKon(0);
    delay(1000);
  }

  printf("Brake ON joint: ALL\n");
  for(i=0;i<2;i++) {
    AllBKon(1);
    delay(1000);
  }

  printf("Brake OFF joint: ALL\n");
  for(i=0;i<1;i++) {
    AllBKon(0);
    delay(1000);
  }

  printf("Brake ON joint: 2\n");
  for(i=0;i<5;i++) {
    BKon(1,1);
    delay(1000);
  }
  BKon(1,0);

  printf("Brake ON joint: 3\n");
  for(i=0;i<5;i++) {
    BKon(2,1);
    delay(1000);
  }
  BKon(2,0);

  printf("Brake ON joint: 4\n");
  for(i=0;i<5;i++) {
    BKon(3,1);
    delay(1000);
  }
  BKon(3,0);

  printf("Brake ON joint: 6\n");
  for(i=0;i<5;i++) {
    BKon(5,1);
    delay(1000);
  }
  BKon(5,0);

  printf("Brake ON joint: 3\n");
  for(i=0;i<5;i++) {
    BKon(2,1);
    delay(1000);
  }

  DioFin();

  return 0;
}
