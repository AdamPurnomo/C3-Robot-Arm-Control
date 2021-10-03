#ifndef _DIO_6464T2_
#define _DIO_6464T2_

// qnx用DAボード（DA16-16L）制御用プログラム
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pcidev.h"

// 関数のプロトタイプ宣言
BOOL DioInit();
void DioFin();
BYTE DioRead_bit(unsigned int ch_num);
BYTE DioRead_byte(unsigned int gr_num);
void DioWrite_all(BYTE outch[32]);
void DioWrite_byte(unsigned int gr_num, BYTE wr_data);
void DioWrite_port(int wr_addr, BYTE wr_data);

#endif
