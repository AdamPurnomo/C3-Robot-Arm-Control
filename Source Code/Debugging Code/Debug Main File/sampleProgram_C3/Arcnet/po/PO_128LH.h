#ifndef _PO_128LH_
#define _PO_128LH_

// qnx�pPO�{�[�h�iPO-128LH�j����p�v���O����
#include "pcidev.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>


// �֐��̃v���g�^�C�v�錾
BOOL PoInit();
void PoFin();
void PoWrite_all( BYTE outch[32]);
void PoWrite_byte( unsigned int gr_num, BYTE wr_data);
void PoWrite_byte_c3kai( unsigned int gr_num, BYTE wr_data);	// 2010/09/03 ��{���񂩂�R�s�[ kamei
void PoWrite_port( int wr_addr, BYTE wr_data);

#endif
