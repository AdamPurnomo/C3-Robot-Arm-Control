#ifndef _PI_128LH_
#define _PI_128LH_

// qnx�pPI�{�[�h�iPI-128LH�j����p�v���O����
#include "./pcidev.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// �֐��̃v���g�^�C�v�錾
BOOL PiInit();
void PiFin();
BYTE PiRead_bit(unsigned int ch_num);
BYTE PiRead_byte(unsigned int gr_num);
//void PiWrite_all(BYTE outch[32]);yoshida deleted.
//void PiWrite_byte(unsigned int gr_num, BYTE wr_data);yoshida deleted.
//void PiWrite_port(int wr_addr, BYTE wr_data);
void PiWrite_port(int wr_addr, BYTE wr_data);

#endif
