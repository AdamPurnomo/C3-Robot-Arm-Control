// INtime用DAボード（DA16-16L）制御用プログラム
#include "DA16_16L.h"

// ボードの情報
#define DEVNAME "DA16-16L"
#define VEN_ID  0x1221  // Vender ID
#define DEV_ID  0x91C3  // デバイスID
#define MAXCH   16      // ボードの最大チャンネル数

// 定数
#define HIGH    1       // DAデータ送信確認用
#define LOW     0       // DAデータ送信確認用

// 変換式 ボルト値をビット数に変換
#define volt2binary1(s) (((int)(1024.*64./20.*(s+10.))) & 0x0000ffff)

// グローバル変数宣言
static struct PCI_dev_t *PCI_dev_da;

// このファイル内のみ使用するプロトタイプ宣言
//int SENDING(int dev_num);

/******************************************/
// 初期化関数
/******************************************/
BOOL DaInit()
{
  int				j;
  unsigned long	ao_sts;
  unsigned short	eepromdata;
  unsigned short	digpot_offset, digpot_gain;
  unsigned long	ao_flg;
  //struct PCI_dev_t PCI_dev_da_tmp;

  PCI_dev_da = (struct PCI_dev_t *)calloc(1,sizeof(struct PCI_dev_t));
  // PCIデバイスの初期設定
  if(PCI_Init_new(0, VEN_ID, DEV_ID, DEVNAME,PCI_dev_da)){
    return FALSE;
  }

  // メインで使うIOアドレスの設定
  PCI_dev_da->uioaddr = PCI_dev_da->ioaddr[0];

  // 動作設定
  // Initialize
  outword(PCI_dev_da->uioaddr + 0x38, 0x00000000);	// ECU Reset
  outword(PCI_dev_da->uioaddr + 0x30, 0x20000000);	// AO Reset
  outword(PCI_dev_da->uioaddr + 0x30, 0x30000000);	// DI Reset
  outword(PCI_dev_da->uioaddr + 0x30, 0x40000000);	// DO Reset
  outword(PCI_dev_da->uioaddr + 0x30, 0x50000000);	// CNT Reset
  outword(PCI_dev_da->uioaddr + 0x30, 0x60000000);	// MEM Reset
  //		printf("Initialize done\n");

  // ECU Setting Destination Source Select
  outword(PCI_dev_da->uioaddr + 0x38, 0x00000003);
  outhword(PCI_dev_da->uioaddr + 0x3C, 0x0020);	// AO Start Condition
  outhword(PCI_dev_da->uioaddr + 0x3E, 0x0180);	// Genearal Commnd
  outword(PCI_dev_da->uioaddr + 0x38, 0x00000003);
  outhword(PCI_dev_da->uioaddr + 0x3C, 0x0022);	// AO Stop Condition
  outhword(PCI_dev_da->uioaddr + 0x3E, 0x0050);	// AOBeforeTriggerSamplingEnd
  outword(PCI_dev_da->uioaddr + 0x38, 0x00000003);
  outhword(PCI_dev_da->uioaddr + 0x3C, 0x0024);	// AO Clock Condition
  outhword(PCI_dev_da->uioaddr + 0x3E, 0x0042);	// InternalCLK
  //		printf("ECU Setting done\n");

  // AO Setting
  outword(PCI_dev_da->uioaddr + 0x30, 0x20000003);	// InternalCLK
  outword(PCI_dev_da->uioaddr + 0x34, 0x0000018F);	// 10usec
  outword(PCI_dev_da->uioaddr + 0x30, 0x2000000C);	// Channel Mode
  outword(PCI_dev_da->uioaddr + 0x34, 0x00000001);	// Multi
  outword(PCI_dev_da->uioaddr + 0x30, 0x20000005);	// Channel Number
  outword(PCI_dev_da->uioaddr + 0x34, 0x0000000F);	// 16ch
  outword(PCI_dev_da->uioaddr + 0x30, 0x20000009);	// Before Trigger Sampling Number
  outword(PCI_dev_da->uioaddr + 0x34, 0x00000000);	// 1 Time
  //		printf("AO Setting done\n");

  // Adjust Data Setting
  outword(PCI_dev_da->uioaddr + 0x30, 0x20000021);	// EEPROM Data Read
  outhword(PCI_dev_da->uioaddr + 0x34, 0x0200);	// 0Ch ±10V
  do{	// EPROMBusy Wait
    ao_sts = inword(PCI_dev_da->uioaddr + 0x0C) & 0x00000200;
  }while(ao_sts != 0x00000000);
  eepromdata = inhword(PCI_dev_da->uioaddr + 0x36);
  digpot_offset = eepromdata & 0x00FF;
  digpot_gain = (eepromdata >> 8) & 0x00FF;
  //		printf("Adjust Data Setting done\n");

  // Digital Potentio-meter Setting
  outword(PCI_dev_da->uioaddr + 0x30 , 0x20000020);	// Digtal Potentio-meter Access
  outhword(PCI_dev_da->uioaddr + 0x34 , 0x0000);	// ±10V Offset
  outhword(PCI_dev_da->uioaddr + 0x36 , digpot_offset);
  do{	// DigPotBusy Wait
    ao_sts = inword(PCI_dev_da->uioaddr + 0x0C) & 0x00000100;
  }while(ao_sts != 0x00000000);
  outword(PCI_dev_da->uioaddr + 0x30, 0x20000020);	// Digtal Potentio-meter Access
  outhword(PCI_dev_da->uioaddr + 0x34, 0x0001);	// ±10V Gain
  outhword(PCI_dev_da->uioaddr + 0x36, digpot_gain);
  do{	// DigPotBusy Wait
    ao_sts = inword(PCI_dev_da->uioaddr + 0x0C) & 0x00000100;
  }while(ao_sts != 0x00000000);
  //		printf("Digital Potentio-meter Setting done\n");

  // AO Output data (0ボルト出力)
  for(j=0;j < 16; j++) {
    outword(PCI_dev_da->uioaddr + 0x08, 0x00008000);
  }

  // AO Sampling
  outword(PCI_dev_da->uioaddr + 0x30, 0x20000001);	// Internal Gate Open
  outword(PCI_dev_da->uioaddr + 0x38, 0x00000005);	// AO Start(General Commnad)
  do{	// AO MotionEndFlag Wait
    outword(PCI_dev_da->uioaddr + 0x38, 0x20000000);
    ao_flg = inword(PCI_dev_da->uioaddr + 0x3C) & 0x80000000;
  }while(ao_flg != 0x80000000);


  return TRUE;
}


/**************************************************/
// 終了関数
/**************************************************/
void DaFin()
{
  // 全チャンネルに0ボルトを出力
  DaOut_zero();

  // メモリの開放
  free((void *)PCI_dev_da);
  //FreeRtMemory(PCI_dev_da);
}
//	printf("%s finished!\n", DEVNAME);


void DaOut_volt(unsigned int ch_num, double volt)
{
  int i;
  unsigned short volt_short;

  for(i=0;i<MAXCH;i++) {
    if(ch_num == i) volt_short= volt2binary1(volt);
    else            volt_short= volt2binary1(0);

    outhword(PCI_dev_da->uioaddr + 0x08,volt_short);
  }

  // AO Sampling
  outword(PCI_dev_da->uioaddr + 0x30, 0x20000001);	// Internal Gate Open
  outword(PCI_dev_da->uioaddr + 0x38, 0x00000005);	// AO Start(General Commnad)
}


// void DaOut_c3kai(double volt[])
// {
//   int i,j;
//   unsigned short volt_short[7];
//   unsigned short tmp[16];
// 
//   j = 0;
//   for(i=0;i<7;i++) {
//       if(i==2) {
//         j++;
//         continue;
//       }
//       volt_short[i] = volt2binary1(volt[i-j]);
//   }
//   volt_short[2] = volt2binary1(volt[6]);
// 
//   for(i=0;i<MAXCH;i++) {
//     if(i < 7) tmp[i] = volt_short[i];
//     else      tmp[i] = volt2binary1(0);
//     printf("tmp[%d] = %d\n",i,tmp[i]);
//   }
// 
//   // // AO Sampling
//   // outword(PCI_dev_da->uioaddr + 0x30, 0x20000001);	// Internal Gate Open
//   // outword(PCI_dev_da->uioaddr + 0x38, 0x00000005);	// AO Start(General Commnad)
// }

void DaOut_c3kai(double volt[])
{
  int i,j;
  unsigned short volt_short[7];

 j = 0;
 for(i=0;i<7;i++) {
     if(i==2) {
       j++;
       continue;
     }
     volt_short[i-j] = volt2binary1(volt[i]);
 }
 volt_short[6] = volt2binary1(volt[2]);

  for(i=0;i<MAXCH;i++) {
    if(i < 7) outhword(PCI_dev_da->uioaddr + 0x08,volt_short[i]);
    else      outhword(PCI_dev_da->uioaddr + 0x08,volt2binary1(0));
  }

  // AO Sampling
  outword(PCI_dev_da->uioaddr + 0x30, 0x20000001);	// Internal Gate Open
  outword(PCI_dev_da->uioaddr + 0x38, 0x00000005);	// AO Start(General Commnad)
}


void DaOut_bin_c3kai(unsigned int bin[])
{
  int i;
  static int count1000 = 0;
  unsigned int buf;
  unsigned short bin_short[MAXCH];

  //2番目の値を6番目に入れて、2番目以降を前の番号にずらす nan
  buf=bin[4];

  bin[4]=bin[6];
  bin[6]=bin[8];
  bin[8]=bin[10];
  bin[10]=bin[12];
  bin[12]=buf;

  buf=bin[5];

  bin[5]=bin[7];
  bin[7]=bin[9];
  bin[9]=bin[11];
  bin[11]=bin[13];
  bin[13]=buf;

  for(i=0;i<MAXCH;i++) {
#ifndef SIMU
    bin_short[i]= bin[i];
#endif
  }

#ifndef SIMU
  for(i=0;i<MAXCH;i++) {
    outhword(PCI_dev_da->uioaddr + 0x08,bin_short[i]);
  }
  // AO Sampling
  outword(PCI_dev_da->uioaddr + 0x30, 0x20000001);	// Internal Gate Open
  outword(PCI_dev_da->uioaddr + 0x38, 0x00000005);	// AO Start(General Commnad)
  //	while(SENDING(dev_num)!= LOW){
  //	//	SENDING(dev_num);
  //		;
  //	}
#endif

#ifndef SIMU
  //	if(count1000%1000==0){
  //		for(i=0;i<MAXCH;i++) {	
  //			printf("DaOut_bin(ch%d) : 0x%4x\n", i, PCI_dev_da[dev_num]->reg[i]);
  //		}
  //	};
#endif

  count1000++;
  /*if(count1000%1000 ==0){
    ;
    }*/

  return;
}


void DaOut_volt_c3kai(double volt[])
{
  int i;
  static int count1000 = 0;
  double buf;
  unsigned short volt_short[MAXCH];

  //2番目の値を6番目に入れて、2番目以降を前の番号にずらす nan
  buf=volt[4];

  volt[4]=volt[6];
  volt[6]=volt[8];
  volt[8]=volt[10];
  volt[10]=volt[12];
  volt[12]=buf;

  buf=volt[5];

  volt[5]=volt[7];
  volt[7]=volt[9];
  volt[9]=volt[11];
  volt[11]=volt[13];
  volt[13]=buf;	

  for(i=0;i<MAXCH; i++) {
    if(volt[i] >= 9.){
      volt[i] = 9.;
    } else if(volt[i] <= -9.){
      volt[i] =-9.;
    }
#ifndef SIMU
    volt_short[i]= volt2binary1(volt[i]);
#endif
  }

#ifndef SIMU
  for(i=0;i<MAXCH;i++) {
    outhword(PCI_dev_da->uioaddr + 0x08,volt_short[i]);
  }
  // AO Sampling
  outword(PCI_dev_da->uioaddr + 0x30, 0x20000001);	// Internal Gate Open
  outword(PCI_dev_da->uioaddr + 0x38, 0x00000005);	// AO Start(General Commnad)
  //	while(SENDING(dev_num)!= LOW){
  //	//	SENDING(dev_num);
  //		;
  //	}
#endif

#ifndef SIMU
  //	for(i=0;i<MAXCH;i++) {
  //		printf("DaOut_volt(ch%d) : %fV (0x%4x)\n", i, volt[i], PCI_dev_da[dev_num]->reg[i]);
  //	}
#endif

  count1000++;
  /*if(count1000%1000 ==0){
    ;
    }*/

  return;
}



/*
// 電圧の出力待ち
int SENDING(int dev_num)
{
int				flag;
unsigned long	ao_flg;

flag = HIGH;

#ifndef SIMU
//	flag = inhword(PCI_dev_da[dev_num]->uioaddr+0x0000)&0x0001;
do{	// AO MotionEndFlag Wait
outword(PCI_dev_da[dev_num]->uioaddr + 0x38, 0x20000000);
ao_flg = inword(PCI_dev_da[dev_num]->uioaddr + 0x3C) & 0x80000000;
}while(ao_flg != 0x80000000);
#endif

flag = LOW;

// printf("%d\n", flag);
return(flag);
}
*/

/***************************************************/
// 0ボルト出力
/***************************************************/
void DaOut_zero()
{
  int i;
  double volt[MAXCH];

  for(i=0;i<MAXCH;i++) {
    volt[i] = 0.;
  }

  DaOut_volt_c3kai(volt);
}

/***************************************************/
// ポートに直接出力する。portnumは2の倍数。
/***************************************************/
void DaWrite_port(int wr_addr, WORD wr_data)
{
  outword(PCI_dev_da->uioaddr + wr_addr, wr_data);

  return;
}
