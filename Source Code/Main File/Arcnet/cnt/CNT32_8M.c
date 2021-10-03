// INtime用CNTボード（CNT32-8M）制御用プログラム
#include "CNT32_8M.h"

// ボードの情報
#define DEVNAME "CNT32-8M"
#define VEN_ID  0x1221  // Vender ID
#define DEV_ID  0xC105  // デバイスID
#define MAXCH   8       // ボードの最大チャンネル数

// グローバル変数宣言
static struct PCI_dev_t *PCI_dev_cnt;

/*****************************************************************/
// 初期化関数
/*****************************************************************/
BOOL CntInit()
{
  int j;
  unsigned long	cntdata;

  PCI_dev_cnt = (struct PCI_dev_t *)calloc(1,sizeof(struct PCI_dev_t));
  // PCIデバイスの初期設定
  if(PCI_Init_new(0, VEN_ID, DEV_ID, DEVNAME,PCI_dev_cnt)){
    return FALSE;
  }

  // メインで使うIOアドレスの設定
  PCI_dev_cnt->uioaddr = PCI_dev_cnt->ioaddr[0];
  //		printf("Main IO ADDRESS is %04x.\n", PCI_dev_cnt[i]->uioaddr);

  // 動作設定
  // 位相差パルス、４逓倍、非同期クリア、カウントアップ、一致検出無効
  // カウンタの選択　入力パルス有効
  // 外部ラッチのクリアなし
  // 割り込みマスク設定　割り込みを許可しない
  // timer off

  // ----- Initialize -----
  // 動作モードを「フィルタ0.1us、CW=UP、2相入力、同期クリア、4逓倍」に設定する
  for(j=0;j<MAXCH;j++) {
    outhword(PCI_dev_cnt->uioaddr + 0x08, 0x0008+j);
    outword(PCI_dev_cnt->uioaddr + 0x0c, 0x00001022);	// 差動
    //	outword(PCI_dev_cnt[i]->uioaddr + 0x0c, 0x00001062);	// TTL
  }

  // プリセットレジスタに最大値の半分を設定する
  for(j=0;j<MAXCH;j++) {
    outhword(PCI_dev_cnt->uioaddr + 0x08, 0x0010+j);
    outword(PCI_dev_cnt->uioaddr + 0x0c, 0x80000000);
  }

  // Z相入力を無効に設定する
  for(j=0;j<MAXCH;j++) {
    outhword(PCI_dev_cnt->uioaddr + 0x08, 0x0030+j);
    outword(PCI_dev_cnt->uioaddr + 0x0c, 0x00000001);
  }

  // プリセットデータを全Chにロードする
  outhword(PCI_dev_cnt->uioaddr + 0x08, 0x003c);
  outword(PCI_dev_cnt->uioaddr + 0x0c, 0x000000ff);

  // カウンタを全Chスタートする
  outhword(PCI_dev_cnt->uioaddr + 0x04, 0x00FF);

  // //----- Test Pulse Output -----
   outhword(PCI_dev_cnt->uioaddr + 0x08, 0x003e);	// テストパルス設定コマンド(未使用の場合は設定不要)
   outword(PCI_dev_cnt->uioaddr + 0x0c, 0x0000);	// テストパルスを内部に出力(未使用の場合は設定不要)

  // ----- Read Data -----
  // カウント値を全チャネルラッチする
  outhword(PCI_dev_cnt->uioaddr + 0x02, 0x00ff);

  for(j=0;j<MAXCH;j++) {
    outhword(PCI_dev_cnt->uioaddr + 0x00, j);	// カウント値の読み出しChを指定
    cntdata = inword(PCI_dev_cnt->uioaddr + 0x00);	// カウント値を読み出し(DWordアクセス以外不可)

    //	outhword(PCI_dev_cnt[i]->uioaddr + 0x08, 0x003f);	// ステータスリードコマンド指定
    //	stsdata = inword(PCI_dev_cnt[i]->uioaddr + 0x0c);	// ステータスを読み出し

    //	printf("CountData  StatusData\n");		// カウント値、ステータス表示
    //	printf("%08ld      %02x\n", cntdata, stsdata);
  }
  return TRUE;
}


/*****************************************************************/
// カウンタボード1枚分の値の読み出し
/*****************************************************************/
/*
void CntRead(int dev_num, DWORD cnt[])
{
	int i;
	static long ticks = 0;
//	unsigned long	stsdata;


	outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x02, 0x00ff);	// 全チャネルラッチ

	for(i=0;i<MAXCH;i++) {
		outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x00, i);	// カウント値の読み出しChを指定
		cnt[i] = inword(PCI_dev_cnt[dev_num]->uioaddr + 0x00);	// カウント値を読み出し(DWordアクセス以外不可)

	//	outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x08, 0x003f);	// ステータスリードコマンド指定
	//	stsdata = inword(PCI_dev_cnt[dev_num]->uioaddr + 0x0c);	// ステータスを読み出し

	//	printf("CountData  StatusData\n");		// カウント値、ステータス表示
	//	printf("%08ld      %02x\n", cnt[i], stsdata);
	}

	return;
}
*/

void CntRead_c3kai(DWORD cnt[])
{
  int i;
  DWORD	buf;

  outhword(PCI_dev_cnt->uioaddr + 0x02, 0x00ff);	// 全チャネルラッチ

  for(i=0;i<MAXCH;i++) {
    if(i == 3)  outhword(PCI_dev_cnt->uioaddr + 0x00, 7);	// カウント値の読み出しChを指定
    else        outhword(PCI_dev_cnt->uioaddr + 0x00, i);	// カウント値の読み出しChを指定 
    cnt[i] = inword(PCI_dev_cnt->uioaddr + 0x00);	// カウント値を読み出し(DWordアクセス以外不可)

    //	outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x08, 0x003f);	// ステータスリードコマンド指定
    //	stsdata = inword(PCI_dev_cnt[dev_num]->uioaddr + 0x0c);	// ステータスを読み出し

    //	printf("CountData  StatusData\n");		// カウント値、ステータス表示
    //	printf("%08ld      %02x\n", cnt[i], stsdata);
  }

  //6番目の値を2番目に入れて、2番目以降を後ろの番号にずらす nan
  buf=cnt[6];

  cnt[6]=cnt[5];
  cnt[5]=cnt[4];
  cnt[4]=cnt[3];
  cnt[3]=cnt[2];
  cnt[2]=buf;

  return;
}
/*****************************************************************/
// カウンタボード1枚分の値の書き込み
/*****************************************************************/
/*
void CntWrite(int dev_num, DWORD cnt[])
{
	int i;

	// カウンタを全Chストップする
	outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x04, 0x0000);

	// プリセットレジスタに値を設定する
	for(i=0;i<MAXCH;i++) {
		outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x08, 0x0010+i);
		outword(PCI_dev_cnt[dev_num]->uioaddr + 0x0c, cnt[i]);
	}

	// プリセットデータを全Chにロードする
	outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x08, 0x003c);
	outword(PCI_dev_cnt[dev_num]->uioaddr + 0x0c, 0x000000ff);

	// カウンタを全Chスタートする
	outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x04, 0x00FF);

	return;
}
*/

void CntWrite_c3kai(DWORD cnt[])
{
  int i;
  DWORD	buf;

  //2番目の値を6番目に入れて、2番目以降を前の番号にずらす nan
  buf=cnt[2];

  cnt[2]=cnt[3];
  cnt[3]=cnt[4];
  cnt[4]=cnt[5];
  cnt[5]=cnt[6];
  cnt[6]=buf;

  // カウンタを全Chストップする
  outhword(PCI_dev_cnt->uioaddr + 0x04, 0x0000);

  // プリセットレジスタに値を設定する
  for(i=0;i<MAXCH;i++) {
    outhword(PCI_dev_cnt->uioaddr + 0x08, 0x0010+i);
    outword(PCI_dev_cnt->uioaddr + 0x0c, cnt[i]);
  }

  // プリセットデータを全Chにロードする
  outhword(PCI_dev_cnt->uioaddr + 0x08, 0x003c);
  outword(PCI_dev_cnt->uioaddr + 0x0c, 0x000000ff);

  // カウンタを全Chスタートする
  outhword(PCI_dev_cnt->uioaddr + 0x04, 0x00FF);

  return;
}


/*****************************************************************/
// 終了関数
/*****************************************************************/
void CntFin()
{
  int i;

  for(i=0;i<4;i++) {
    outbyte(PCI_dev_cnt->ioaddr[i] + 0x06, 0x03);
    outbyte(PCI_dev_cnt->ioaddr[i] + 0x16, 0x03);
  }
  free((void *)PCI_dev_cnt);

  //	printf("%s finished!\n", DEVNAME);
  return ;
}


/*****************************************************************/
// 任意のカウンタ値をセットする。
/*****************************************************************/
void CntSet(int ch, DWORD cnt)
{
	// 0,2,4,6軸
	if(ch%2 == 0) {
		outbyte(PCI_dev_cnt->ioaddr[ch/2] + 0x00, cnt);
		outbyte(PCI_dev_cnt->ioaddr[ch/2] + 0x01, cnt>>8);
		outbyte(PCI_dev_cnt->ioaddr[ch/2] + 0x02, cnt>>16);
	}else {
		outbyte(PCI_dev_cnt->ioaddr[ch/2] + 0x10, cnt);
		outbyte(PCI_dev_cnt->ioaddr[ch/2] + 0x11, cnt>>8);
		outbyte(PCI_dev_cnt->ioaddr[ch/2] + 0x12, cnt>>16);
	}
}

