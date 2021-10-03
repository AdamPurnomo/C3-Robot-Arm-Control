#ifndef _PO_128LH_
#define _PO_128LH_

// qnx用POボード（PO-128LH）制御用プログラム
#include "pcidev.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>


// 関数のプロトタイプ宣言
BOOL PoInit();
void PoFin();
void PoWrite_all( BYTE outch[32]);
void PoWrite_byte( unsigned int gr_num, BYTE wr_data);
void PoWrite_byte_c3kai( unsigned int gr_num, BYTE wr_data);	// 2010/09/03 南本さんからコピー kamei
void PoWrite_port( int wr_addr, BYTE wr_data);

#endif
