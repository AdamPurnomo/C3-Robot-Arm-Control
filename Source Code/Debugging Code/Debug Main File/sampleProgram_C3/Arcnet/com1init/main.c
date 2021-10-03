#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>
#include "pcidev.h"
#include "REX_PCI60D.h"
#include "../dio/DIO_6464T2.h"

int main()
{
  int           i,j;
  BYTE          Sen;
  int           err;
  BYTE          rxdata[10];             // unsigned char から charに変更 20181020 yoshinaga
  const int     max_rx_bytes = 8;
	signed short  AbsPos[7];		// 多回転位置

  i = 0;
	err = 0;

  ThreadCtl(_NTO_TCTL_IO, 0);

  //ボードの初期化
  if(DioInit() == FALSE) {
    printf("DioInit ERROR!\n");
    return 1;
  }
  else printf("DioInit OK!\n");

  printf("\n");
  printf("com1init\n");
  Com1Init();

  if (Com1Init() == FALSE) {
    printf("Com1Init ERROR!\n");
  }
  else printf("Com1Init OK!");

  for(j=0;j<max_rx_bytes+1;j++) {     // 変数の初期化
    *(rxdata+j) = 0x00;
  }

  DioWrite_byte(0x06, 0x00);          // SELを非選択状態にする
  DioWrite_byte(0x04, 0xFF);          // SENを全てLowにする 負論理

  Sen = 0xFF;                         // SENは１台ずつHighにする
  DioWrite_byte(0x04, ~Sen);          // SENを出力する 負論理
  delay(100);

  printf("FIFO clear\n");
  Com1FifoClr();
  DioWrite_byte(0x06, ~i);            // SEL＝0x0～0x6 負論理
  printf("/SEL output\n");

  err |= Com1Rx(rxdata, max_rx_bytes);
  printf("COM1RX\n");
  DioWrite_byte(0x06, 0x00);          // SELを非選択状態にする
  // 受信データをチェックする
  if(((*(rxdata+0)!=0x50) || (*(rxdata+7)!=0x0D))
      || ((*(rxdata+1)!=0x2B) && (*(rxdata+1)!=0x2D))) {	// '0x50'，'0x0D'などアスキーコード
    err = 1;
    printf("RxData error!\n");
  }
  for(j=2; j<7; j++) {
    if( (*(rxdata+j) < 0x30) || (*(rxdata+j) > 0x39) ) {	// '0x30'，'0x39'などアスキーコード
      err = 1;
      printf("RxData error!\n");
    }
  }

  AbsPos[i] = atoi((char*)rxdata+1);      // AbsPos0[] : 多回転量

  // 多回転位置をチェックする
  if( (AbsPos[i] < -38) || (AbsPos[i] > 38) ) {
    err = 1;
    printf("AbsPos error!\n");
  }

  printf("AbsPos[%d] = %d\n",i,AbsPos[i]);

  // struct PCI_dev_t buff;
  // PCI_Init_new(0, 0x13a8, 0x152, 0,  &buff);
  // printf("buff.ven_id:%lx\n", buff.ven_id);
  // printf("buff.dev_id:%lx\n", buff.dev_id);
  // printf("buff.uioaddr:%lx\n", buff.uioaddr);
  // printf("buff.rev_id:%x\n", buff.rev_id);
  // printf("buff.ioaddr[0]:%lx\n", buff.ioaddr[0]);
  // printf("buff.ioaddr[1]:%lx\n", buff.ioaddr[1]);
  // printf("buff.ioaddr[2]:%lx\n", buff.ioaddr[2]);
  // printf("buff.ioaddr[3]:%lx\n", buff.ioaddr[3]);
  // printf("buff.ioaddr[4]:%lx\n", buff.ioaddr[4]);
  // printf("buff.ioaddr[5]:%lx\n", buff.ioaddr[5]);

  DioFin();

  return 0;
}
