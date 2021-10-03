#ifndef _REXPCI60D_
#define _REXPCI60D_
// qnx用COM1制御用プログラム
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pcidev.h"
#include <sys/mman.h>

// 関数のプロトタイプ宣言
BOOL Com1Init();
BOOL Com1Tx(BYTE* txdata, int txlength);
BOOL Com1Rx(BYTE* rxdata, int rxlength);
void Com1FifoClr();

#endif
