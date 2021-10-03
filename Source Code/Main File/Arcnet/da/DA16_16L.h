#ifndef _DA16_16L_
#define _DA16_16L_

// qnx用DAボード（DA16-16L）制御用プログラム
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pcidev.h"

// 関数のプロトタイプ宣言
BOOL DaInit();
void DaFin();
void DaOut_volt(unsigned int ch_num, double volt);
void DaOut_c3kai(double volt[]);
void DaOut_bin_c3kai(unsigned int bin[]);
void DaOut_volt_c3kai(double volt[]);
void DaOut_zero();
void DaWrite_port(int wr_addr, WORD wr_data);

#endif
