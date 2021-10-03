// INtime�p��≏DIO�{�[�h�iDIO-6464T2�j����p�v���O����
#include "./PI_128LH.h"

//#include <rt.h>//yoshida(begin)
//#include <pcibus.h>
#include <hw/inout.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>//yoshida(end)

// �{�[�h�̏��
#define DEVNAME "PI-128LH"
#define VEN_ID  0x1221  // Vender ID
#define DEV_ID  0xA1D2  // �f�o�C�XID
#define MAXINCH		128	// IN���̃`�����l����
#define MAXOUTCH	0	// OUT���̃`�����l����
#define DEV_NUM		1	// �g�p����{�[�h�̖���

// pci_dev.c
//extern struct PCI_dev_t *PCI_Init_new(int dev_num, WORD ven_id, WORD dev_id, char *dev_name);

// �O���[�o���ϐ��錾
static struct PCI_dev_t *PCI_dev_pi;

/*****************************************************************/
// �������֐�
/*****************************************************************/

BOOL PiInit()
{
  int  j;

  // PCI�f�o�C�X�������̍��}
  //	printf("Searching %s....\n", DEVNAME);

  // PCI�f�o�C�X�̏����ݒ�
  PCI_dev_pi = (struct PCI_dev_t *)calloc(1, sizeof(struct PCI_dev_t));

  if (PCI_Init_new(0, VEN_ID, DEV_ID, DEVNAME, PCI_dev_pi)){
    return FALSE;
  }
  // ���C���Ŏg��IO�A�h���X�̐ݒ�
  PCI_dev_pi->uioaddr = PCI_dev_pi->ioaddr[0];
  //printf("Main IO ADDRESS is %04x.\n", PCI_dev_pi->uioaddr);


  // ����ݒ�

  // �f�W�^���t�B���^�i���g�p�j
  // ���荞�݃}�X�N�i���g�p�j
  // ���荞�݃G�b�W�I���i���g�p�j
  // �C���^�[�o���^�C�}�i���g�p�j


  // �O����H�͂��ׂ�OFF(pullup��R�ɂ��High)�ɂ��Ă���
  for (j = 0; j<8; j++) {
    outbyte(PCI_dev_pi->uioaddr + 0x08 + j, 0x00);
  }


  return TRUE;
}


/*****************************************************************/
// �I���֐�
/*****************************************************************/
void PiFin()
{
  int  j;

  // �o�͂͂��ׂ�OFF(pullup��R�ɂ��High)�ɂ��Ă���
  for (j = 0; j<8; j++) {
    outbyte(PCI_dev_pi->uioaddr + 0x08 + j, 0x00);
  }

  // �������[�̉��
  free((void *)PCI_dev_pi);

  //	printf("%s finished.\n", DEVNAME);

  return;

}

/*****************************************************************/
// �w��̂P�r�b�g��ǂݍ���(���_��)
// dev_num;�{�[�hNO
// ch_num;���͂������`�����l���ԍ�(0-63)
/*****************************************************************/
BYTE PiRead_bit( unsigned int ch_num)
{
  BYTE port;
  BYTE onoff;

  port = inbyte(PCI_dev_pi->uioaddr + 0x00 + (ch_num >> 3));
  onoff = port >> (ch_num - (ch_num & 0xF8));
  //	printf("PiRead_bit : port=%d, onoff=%d\n", port, onoff);
  return onoff;
}

/*****************************************************************/
// �w��̂P�o�C�g��ǂݍ���
// dev_num;�{�[�hNO
// gr_num;���͂������O���[�v�ԍ�(0-15)
/*****************************************************************/
BYTE PiRead_byte( unsigned int gr_num)
{
  BYTE port;

  port = inbyte(PCI_dev_pi->uioaddr + 0x00 + gr_num);

  return port;
}

/*****************************************************************/
// �|�[�g�ɒ��ڏo�͂���B
/*****************************************************************/
void PiWrite_port( int wr_addr, BYTE wr_data)
{
  outbyte(PCI_dev_pi->uioaddr + wr_addr, wr_data);

  return;
}
