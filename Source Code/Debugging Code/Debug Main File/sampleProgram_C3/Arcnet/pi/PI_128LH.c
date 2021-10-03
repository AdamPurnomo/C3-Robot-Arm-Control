// INtime用非絶縁DIOボード（DIO-6464T2）制御用プログラム
#include "./PI_128LH.h"

//#include <rt.h>//yoshida(begin)
//#include <pcibus.h>
#include <hw/inout.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>//yoshida(end)

// ボードの情報
#define DEVNAME "PI-128LH"
#define VEN_ID  0x1221  // Vender ID
#define DEV_ID  0xA1D2  // デバイスID
#define MAXINCH		128	// IN側のチャンネル数
#define MAXOUTCH	0	// OUT側のチャンネル数
#define DEV_NUM		1	// 使用するボードの枚数

// pci_dev.c
//extern struct PCI_dev_t *PCI_Init_new(int dev_num, WORD ven_id, WORD dev_id, char *dev_name);

// グローバル変数宣言
static struct PCI_dev_t *PCI_dev_pi;

/*****************************************************************/
// 初期化関数
/*****************************************************************/

BOOL PiInit()
{
  int  j;

  // PCIデバイス初期化の合図
  //	printf("Searching %s....\n", DEVNAME);

  // PCIデバイスの初期設定
  PCI_dev_pi = (struct PCI_dev_t *)calloc(1, sizeof(struct PCI_dev_t));

  if (PCI_Init_new(0, VEN_ID, DEV_ID, DEVNAME, PCI_dev_pi)){
    return FALSE;
  }
  // メインで使うIOアドレスの設定
  PCI_dev_pi->uioaddr = PCI_dev_pi->ioaddr[0];
  //printf("Main IO ADDRESS is %04x.\n", PCI_dev_pi->uioaddr);


  // 動作設定

  // デジタルフィルタ（未使用）
  // 割り込みマスク（未使用）
  // 割り込みエッジ選択（未使用）
  // インターバルタイマ（未使用）


  // 外部回路はすべてOFF(pullup抵抗によりHigh)にしておく
  for (j = 0; j<8; j++) {
    outbyte(PCI_dev_pi->uioaddr + 0x08 + j, 0x00);
  }


  return TRUE;
}


/*****************************************************************/
// 終了関数
/*****************************************************************/
void PiFin()
{
  int  j;

  // 出力はすべてOFF(pullup抵抗によりHigh)にしておく
  for (j = 0; j<8; j++) {
    outbyte(PCI_dev_pi->uioaddr + 0x08 + j, 0x00);
  }

  // メモリーの解放
  free((void *)PCI_dev_pi);

  //	printf("%s finished.\n", DEVNAME);

  return;

}

/*****************************************************************/
// 指定の１ビットを読み込む(負論理)
// dev_num;ボードNO
// ch_num;入力したいチャンネル番号(0-63)
/*****************************************************************/
BYTE PiRead_bit( unsigned int ch_num)
{
  BYTE port;
  BYTE onoff;

  port = inbyte(PCI_dev_pi->uioaddr + 0x00 + (ch_num >> 3));
  onoff = port >> (ch_num - (ch_num & 0xF8));
  //	printf("PiRead_bit : port=%d, onoff=%d\n", port, onoff);
  return onoff;
}

/*****************************************************************/
// 指定の１バイトを読み込む
// dev_num;ボードNO
// gr_num;入力したいグループ番号(0-15)
/*****************************************************************/
BYTE PiRead_byte( unsigned int gr_num)
{
  BYTE port;

  port = inbyte(PCI_dev_pi->uioaddr + 0x00 + gr_num);

  return port;
}

/*****************************************************************/
// ポートに直接出力する。
/*****************************************************************/
void PiWrite_port( int wr_addr, BYTE wr_data)
{
  outbyte(PCI_dev_pi->uioaddr + wr_addr, wr_data);

  return;
}
