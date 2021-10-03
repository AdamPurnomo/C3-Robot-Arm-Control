// INtime用非絶縁DIOボード（DIO-6464T2）制御用プログラム
#include "PO_128LH.h"

// ボードの情報
#define DEVNAME "PO-128LH"
#define VEN_ID	0x1221	// Vender ID
#define DEV_ID	0xA1E2	// デバイスID
#define MAXINCH		0	// IN側のチャンネル数
#define MAXOUTCH	128	// OUT側のチャンネル数
#define DEV_NUM		1	// 使用するボードの枚数


// グローバル変数宣言
static struct PCI_dev_t *PCI_dev_po;

/*****************************************************************/
// 初期化関数
/*****************************************************************/

BOOL PoInit()
{
  int  j;

  // PCIデバイス初期化の合図
  //	printf("Searching %s....\n", DEVNAME);
  PCI_dev_po = (struct PCI_dev_t *)calloc(1, sizeof(struct PCI_dev_t));
  // PCIデバイスの初期設定
  if (PCI_Init_new(0, VEN_ID, DEV_ID, DEVNAME, PCI_dev_po)){
    return FALSE;
  }
  // メインで使うIOアドレスの設定
  PCI_dev_po->uioaddr = PCI_dev_po->ioaddr[0];
  //		printf("Main IO ADDRESS is %04x.\n", PCI_dev_po[i]->uioaddr);


  // 動作設定
  // デジタルフィルタ（未使用）
  // 割り込みマスク（未使用）
  // 割り込みエッジ選択（未使用）
  // インターバルタイマ（未使用）

  // 外部回路はすべてOFF(pullup抵抗によりHigh)にしておく
  for (j = 0; j<16; j++) {
    outbyte(PCI_dev_po->uioaddr + 0x00 + j, 0x00);
  }

  return TRUE;
}


/*****************************************************************/
// 終了関数
/*****************************************************************/
void PoFin()
{
  int  j;

  // 出力はすべてOFF(pullup抵抗によりHigh)にしておく
  for (j = 0; j<8; j++) {
    outbyte(PCI_dev_po->uioaddr + 0x00 + j, 0x00);
  }


  // メモリーの解放
  free((void *)PCI_dev_po);

  //	printf("%s finished.\n", DEVNAME);

  return;

}

/*****************************************************************/
// ボードBNOのOUTnに書き出す
// dev_num;ボードNO
// outsignal;出力したいport番号
// outch;各チャンネルのON（1）、OFF(0)の配列（128チャンネル）
/*****************************************************************/
void PoWrite_all( BYTE outch[])
{
  // 	int i,j;
  int i;
  const int p_num = 16;	// 16つのポートがある
  BYTE port[16];

  for (i = 0; i<p_num; i++) {
    port[i] = 0x00;
    port[i] = outch[0 + i * 8] * 1 + outch[1 + i * 8] * (2) + outch[2 + i * 8] * (4)
      + outch[3 + i * 8] * (8) + outch[4 + i * 8] * (16) + outch[5 + i * 8] * (32)
      + outch[6 + i * 8] * (64) + outch[7 + i * 8] * (128);
  }

  for (i = 0; i<p_num; i++) {
#ifndef SIMU
    outbyte(PCI_dev_po->uioaddr + 0x00 + i, port[i]);
#endif
  }

}

/*****************************************************************/
// ボードBNOのOUTnに書き出す
// dev_num;ボードNO
// gr_num;出力したいグループ番号(0-15)
// wr_data;８ビットの1(High）、0(Low)の配列
/*****************************************************************/
void PoWrite_byte( unsigned int gr_num, BYTE wr_data)
{

#ifndef SIMU
  outbyte(PCI_dev_po->uioaddr + 0x00 + gr_num, wr_data);
#endif

}

// 2010/09/03 南本さんからコピー kamei
void PoWrite_byte_c3kai( unsigned int gr_num, BYTE wr_data)
{
  BYTE	write_data;
  BYTE	b0, b1;

  //2ビット目を6ビット目に移動して、2ビット目以降を前にずらす
  b0 = (wr_data >> 1) & 0x3C;	//zz65_43zz
  b1 = wr_data & 0x83;		//7zzz_zz10
  b0 = b0 | b1;				//7z65_4310

  b1 = (wr_data << 4) & 0x40;	//z2zz_zzzz
  b0 = b0 | b1;				//7265_4310

  write_data = b0;

#ifndef SIMU
  outbyte(PCI_dev_po->uioaddr + 0x00 + gr_num, write_data);
#endif

}

/*****************************************************************/
// ポートに直接出力する。
/*****************************************************************/
void PoWrite_port( int wr_addr, BYTE wr_data)
{
  outbyte(PCI_dev_po->uioaddr + wr_addr, wr_data);

  return;
}

