#ifndef _DIO_6464T2_
#define _DIO_6464T2_

// qnx�pDA�{�[�h�iDA16-16L�j����p�v���O����
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pcidev.h"

// �֐��̃v���g�^�C�v�錾
BOOL DioInit();
void DioFin();
BYTE DioRead_bit(unsigned int ch_num);
BYTE DioRead_byte(unsigned int gr_num);
void DioWrite_all(BYTE outch[32]);
void DioWrite_byte(unsigned int gr_num, BYTE wr_data);
void DioWrite_port(int wr_addr, BYTE wr_data);

#endif
