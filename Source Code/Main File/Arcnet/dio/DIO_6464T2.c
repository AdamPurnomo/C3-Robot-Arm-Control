// INtime用非絶縁DIOボード（DIO-6464T2）制御用プログラム
#include "DIO_6464T2.h"

// ボードの情報
#define DEVNAME "DIO-6464T2"
#define VEN_ID  0x1221  // Vender ID
#define DEV_ID  0xB122  // デバイスID
#define MAXINCH		64	// IN側のチャンネル数
#define MAXOUTCH	64	// OUT側のチャンネル数

// グローバル変数宣言
static struct PCI_dev_t *PCI_dev_dio;

// このファイル内のみで使用するプロトタイプ宣言
// PCI_dev_t *PCI_Init_DIO(int dev_num);

/*****************************************************************/
// 初期化関数
/*****************************************************************/

BOOL DioInit()
{
  int j;

  // PCIデバイス初期化の合図
  //	printf("Searching %s....\n", DEVNAME);

  PCI_dev_dio = (struct PCI_dev_t *)calloc(1,sizeof(struct PCI_dev_t));
  // PCIデバイスの初期設定
  if(PCI_Init_new(0,VEN_ID, DEV_ID, DEVNAME,PCI_dev_dio)){
    return FALSE;
  }
  // メインで使うIOアドレスの設定
  PCI_dev_dio->uioaddr = PCI_dev_dio->ioaddr[0];
  //		printf("Main IO ADDRESS is %04x.\n", PCI_dev_dio->uioaddr);

  // 動作設定
  // デジタルフィルタ（未使用）
  // 割り込みマスク（未使用）
  // 割り込みエッジ選択（未使用）
  // インターバルタイマ（未使用）

  // 出力回路はオープンコレクタで負論理
  // ONはLowに、OFFはpullup抵抗によりHighになる
  // すべてON(Low)にしておく
  for(j=0;j<8;j++) {
    outbyte(PCI_dev_dio->uioaddr + 0x08+j, 0xFF);
  }

  // ブレーキ信号はOFFにしておく
  outbyte(PCI_dev_dio->uioaddr + 0x08, 0x00);

  return TRUE;
}


/*****************************************************************/
// 終了関数
/*****************************************************************/
void DioFin()
{
  int j;

  // 出力はすべてON(Low)にしておく
  for(j=0;j<8;j++)
    outbyte(PCI_dev_dio->uioaddr + 0x08+j, 0xFF);

  // ブレーキ信号はOFFにしておく
  outbyte(PCI_dev_dio->uioaddr + 0x08, 0x00);

  // メモリーの解放
  free((void *)PCI_dev_dio);
  //	printf("%s finished.\n", DEVNAME);

  return ;
}
/**************************************************/
/* DIO -> 入力ポート　"BaseAddress + 0x00"        */
/*        出力ポート  "BaseAddress + 0x08" kamei  */
/**************************************************/

/*****************************************************************/
// 指定の１ビットを読み込む(負論理)
// dev_num;ボードNO
// ch_num;入力したいチャンネル番号(0-63)
/*****************************************************************/
BYTE DioRead_bit(unsigned int ch_num)
{
  BYTE port;
  BYTE onoff;

  port = inbyte(PCI_dev_dio->uioaddr + 0x00 + (ch_num>>3));
  onoff = port >> (ch_num - (ch_num&0xF8));
  //	printf("DioRead_bit : port=%d, onoff=%d\n", port, onoff);
  return onoff;
}

/*****************************************************************/
// 指定の１バイトを読み込む
// dev_num;ボードNO
// gr_num;入力したいグループ番号(0-7)
/*****************************************************************/
BYTE DioRead_byte(unsigned int gr_num)
{
  BYTE port;

  port = inbyte(PCI_dev_dio->uioaddr + 0x00 + gr_num);
  return port;
}

/*****************************************************************/
// ボードBNOのOUTnに書き出す
// dev_num;ボードNO
// outsignal;出力したいport番号
// outch;各チャンネルのON（1）、OFF(0)の配列（64チャンネル）
/*****************************************************************/
void DioWrite_all(BYTE outch[])
{
  // 	int i,j;
  int i;
  const int p_num = 8;	// 8つのポートがある
  BYTE port[8];

  for(i=0;i<p_num;i++) {
    port[i] = 0x00;
    port[i] = outch[0+i*8]*1 + outch[1+i*8]*(2) + outch[2+i*8]*(4) 
      + outch[3+i*8]*(8) + outch[4+i*8]*(16) + outch[5+i*8]*(32)
      + outch[6+i*8]*(64) + outch[7+i*8]*(128);
  }

  for(i=0;i<p_num;i++) {
#ifndef SIMU
    outbyte(PCI_dev_dio->uioaddr + 0x08 + i, port[i]);
#endif
  }

}

/*****************************************************************/
// ボードBNOのOUTnに書き出す
// dev_num;ボードNO
// gr_num;出力したいグループ番号(0-7)
// wr_data;８ビットの1(High）、0(Low)の配列
/*****************************************************************/
void DioWrite_byte(unsigned int gr_num, BYTE wr_data)
{

#ifndef SIMU
  outbyte(PCI_dev_dio->uioaddr + 0x08 + gr_num, wr_data);
#endif

}

/*****************************************************************/
// ポートに直接出力する。
/*****************************************************************/
void DioWrite_port(int wr_addr, BYTE wr_data)
{
  outbyte(PCI_dev_dio->uioaddr + wr_addr, wr_data);

  return;
}


