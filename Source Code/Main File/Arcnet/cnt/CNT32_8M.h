#ifndef _CNT32_8M_
#define _CNT32_8M_

// qnx用CNTボード（CNT32-8M）制御用プログラム
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pcidev.h"
#include <hw/inout.h>

BOOL CntInit();

void CntRead_c3kai(DWORD cnt[]);
void CntWrite_c3kai(DWORD cnt[]);

void CntFin();
void CntSet(int ch, DWORD cnt);

#endif
