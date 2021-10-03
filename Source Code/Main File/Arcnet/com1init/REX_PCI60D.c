// qnx用REX-PCI60D(RS232Cシリアル通信)制御用プログラム
#include "REX_PCI60D.h"

#ifndef PortAddress
#define PortAddress (0x8000)
#endif
/*---portaddress kobari*/

#define Delay() (void)inbyte(PortAddress+0x07)//koko
;
// 関数のプロトタイプ宣言
//BOOL Com1Init();
//BOOL Com1Tx(BYTE* txdata, int txlength);
//BOOL Com1Rx(BYTE* rxdata, int rxlength);
//void Com1FifoClr();

/*****************************************************************/
// 初期化関数
/*****************************************************************/

BOOL Com1Init()
{
  // COM1(シリアルポート)の初期化
  // RS-232Cの端子電圧は負論理、LSBファースト
  WORD address;

  address = mmap_device_io(1,PortAddress);
  outbyte(address+ 0x003, 0x80);				// LCRレジスタDLAビット//koko
  printf("LCR = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x003)));//koko
  Delay();

  // 通信速度9600bps
  outbyte(mmap_device_io(1,PortAddress + 0x000), 0x00);				// THR/RBR(Divisor L)レジスタ//koko
  //outbyte(PortAddress+0x000, 0x0C);				// THR/RBR(Divisor L)レジスタ//koko
  printf("THR(Divisor L) = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x000)));//koko
  Delay();

  outbyte(mmap_device_io(1,PortAddress + 0x001), 0x00);				// IER(Divisor H)レジスタ//koko
  printf("IER(Divisor H) = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x001)));//koko
  Delay();

  // outbyte(PortAddress+0x003, 0x13);			// LCRレジスタDLAビット Parityなし8bit//koko
  outbyte(mmap_device_io(1,PortAddress + 0x003), 0x1A);				// LCRレジスタDLAビット Parityあり7bit//koko
  printf("LCR = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x003)));//koko
  Delay();

  outbyte(mmap_device_io(1,PortAddress + 0x001), 0x00);				// IERレジスタ 割り込み禁止//koko
  printf("IER = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x001)));//koko
  Delay();

  outbyte(mmap_device_io(1,PortAddress + 0x004), 0x00);				// MCRレジスタ//koko
  printf("MCR = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x004)));//koko
  Delay();

  outbyte(mmap_device_io(1,PortAddress + 0x002), 0x87);				// FCR(IIR)レジスタ W only//koko
  printf("IIR(FCR) = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x002)));//koko
  Delay();

  outbyte(mmap_device_io(1,PortAddress + 0x004), 0x03);				// MCRレジスタ RTS,DTR//koko
  printf("MCR = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x004)));//koko
  Delay();

  //	inbyte(PortAddress+0x005);					// LSRレジスタ ステータス//koko

  return TRUE;
}

BOOL Com1Tx(BYTE* txdata, int txlength)
{
  int i;
  printf("%x\n", (unsigned int)txdata);
  i=0;
  while(i < txlength) {
    if ((inbyte(mmap_device_io(1,PortAddress + 0x006)) & 0x10)) {	// MSRレジスタCTSビット//koko
      outbyte(mmap_device_io(1,PortAddress + 0x000), txdata[i]);//koko
      i++;
    }
    else {
      ;
    }
    Delay();						// MSRレジスタCTSビット待ち
    // printf("i:%i\n", i);
    // printf("status %x\n", inbyte(mmap_device_io(1, PortAddress + 0x006)));
  }

  printf("%x\n", (unsigned int)txdata);
  return TRUE;
}

BOOL Com1Rx(BYTE* rxdata, int rxlength)
{
  int		i, j;
  BOOL	err;
  BYTE	rxstatus;

  err       = 0;
  i         = 0;
  j         = 0;
  rxstatus  = 0;

  while(i < rxlength) {
    rxstatus = inbyte(mmap_device_io(1,PortAddress + 0x05));		// LSRレジスタ//koko
    if(rxstatus & 0x08) {				// FramingErrビット
      *(rxdata + i) = inbyte(mmap_device_io(1,PortAddress + 0x00));	// RBRレジスタ//koko
      printf("FramingErr at byte No.%d\n", i);
      err = 1;
    }
    else if(rxstatus & 0x04) {			// ParityErrビット
      *(rxdata + i) = inbyte(mmap_device_io(1,PortAddress + 0x00));	// RBRレジスタ//koko
      printf("ParityErr at byte No.%d\n", i);
      err = 1;
    }
    else if(rxstatus & 0x02) {			// FifoOverflowビット
      *(rxdata + i) = inbyte(mmap_device_io(1,PortAddress + 0x00));	// RBRレジスタ//koko
      printf("FifoOverflow at byte No.%d\n", i);
      err = 1;
    }
    else if(rxstatus & 0x01) {			// DataReadyビット
      *(rxdata + i) = inbyte(mmap_device_io(1,PortAddress + 0x00));	// RBRレジスタ//koko
      j = 0;
      i++;
    }
    else if((i>0) && (j>200)) {			// タイムアウト
      i = rxlength;
    }
    else if((i==0) && (j>1000)) {		// タイムアウト
      printf("TimeoutErr at byte No.%d0\n", i);
      printf("%x\n",rxstatus);
      printf("%x\n",PortAddress);
      err = 1;
    }
    else {
      j++;
    }
    Delay();						// 1ms待つ
    if(err) break;
  }
  printf("rxstatus %x\n", (unsigned int)rxstatus);
  printf("rxdata %x\n",(unsigned int)rxdata);
  return err;
}

void Com1FifoClr()
{
  outbyte(mmap_device_io(1,PortAddress + 0x002), 0x87);			// FCRレジスタ FIFOクリア//koko
  Delay();
  inbyte(mmap_device_io(1,PortAddress + 0x005));				// LSRレジスタ ステータス//koko
  inbyte(mmap_device_io(1,PortAddress + 0x000));				// RBRレジスタ 受信データ//koko
}


