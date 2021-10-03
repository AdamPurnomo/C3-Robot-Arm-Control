// INtime�p��≏DIO�{�[�h�iDIO-6464T2�j����p�v���O����
#include "PO_128LH.h"

// �{�[�h�̏��
#define DEVNAME "PO-128LH"
#define VEN_ID	0x1221	// Vender ID
#define DEV_ID	0xA1E2	// �f�o�C�XID
#define MAXINCH		0	// IN���̃`�����l����
#define MAXOUTCH	128	// OUT���̃`�����l����
#define DEV_NUM		1	// �g�p����{�[�h�̖���


// �O���[�o���ϐ��錾
static struct PCI_dev_t *PCI_dev_po;

/*****************************************************************/
// �������֐�
/*****************************************************************/

BOOL PoInit()
{
  int  j;

  // PCI�f�o�C�X�������̍��}
  //	printf("Searching %s....\n", DEVNAME);
  PCI_dev_po = (struct PCI_dev_t *)calloc(1, sizeof(struct PCI_dev_t));
  // PCI�f�o�C�X�̏����ݒ�
  if (PCI_Init_new(0, VEN_ID, DEV_ID, DEVNAME, PCI_dev_po)){
    return FALSE;
  }
  // ���C���Ŏg��IO�A�h���X�̐ݒ�
  PCI_dev_po->uioaddr = PCI_dev_po->ioaddr[0];
  //		printf("Main IO ADDRESS is %04x.\n", PCI_dev_po[i]->uioaddr);


  // ����ݒ�
  // �f�W�^���t�B���^�i���g�p�j
  // ���荞�݃}�X�N�i���g�p�j
  // ���荞�݃G�b�W�I���i���g�p�j
  // �C���^�[�o���^�C�}�i���g�p�j

  // �O����H�͂��ׂ�OFF(pullup��R�ɂ��High)�ɂ��Ă���
  for (j = 0; j<16; j++) {
    outbyte(PCI_dev_po->uioaddr + 0x00 + j, 0x00);
  }

  return TRUE;
}


/*****************************************************************/
// �I���֐�
/*****************************************************************/
void PoFin()
{
  int  j;

  // �o�͂͂��ׂ�OFF(pullup��R�ɂ��High)�ɂ��Ă���
  for (j = 0; j<8; j++) {
    outbyte(PCI_dev_po->uioaddr + 0x00 + j, 0x00);
  }


  // �������[�̉��
  free((void *)PCI_dev_po);

  //	printf("%s finished.\n", DEVNAME);

  return;

}

/*****************************************************************/
// �{�[�hBNO��OUTn�ɏ����o��
// dev_num;�{�[�hNO
// outsignal;�o�͂�����port�ԍ�
// outch;�e�`�����l����ON�i1�j�AOFF(0)�̔z��i128�`�����l���j
/*****************************************************************/
void PoWrite_all( BYTE outch[])
{
  // 	int i,j;
  int i;
  const int p_num = 16;	// 16�̃|�[�g������
  BYTE port[16];

  for (i = 0; i<p_num; i++) {
    port[i] = 0x00;
    port[i] = outch[0 + i * 8] * 1 + outch[1 + i * 8] * (2) + outch[2 + i * 8] * (4)
      + outch[3 + i * 8] * (8) + outch[4 + i * 8] * (16) + outch[5 + i * 8] * (32)
      + outch[6 + i * 8] * (64) + outch[7 + i * 8] * (128);
  }

  for (i = 0; i<p_num; i++) {
#ifndef SIMU
    outbyte(PCI_dev_po->uioaddr + 0x00 + i, port[i]);
#endif
  }

}

/*****************************************************************/
// �{�[�hBNO��OUTn�ɏ����o��
// dev_num;�{�[�hNO
// gr_num;�o�͂������O���[�v�ԍ�(0-15)
// wr_data;�W�r�b�g��1(High�j�A0(Low)�̔z��
/*****************************************************************/
void PoWrite_byte( unsigned int gr_num, BYTE wr_data)
{

#ifndef SIMU
  outbyte(PCI_dev_po->uioaddr + 0x00 + gr_num, wr_data);
#endif

}

// 2010/09/03 ��{���񂩂�R�s�[ kamei
void PoWrite_byte_c3kai( unsigned int gr_num, BYTE wr_data)
{
  BYTE	write_data;
  BYTE	b0, b1;

  //2�r�b�g�ڂ�6�r�b�g�ڂɈړ����āA2�r�b�g�ڈȍ~��O�ɂ��炷
  b0 = (wr_data >> 1) & 0x3C;	//zz65_43zz
  b1 = wr_data & 0x83;		//7zzz_zz10
  b0 = b0 | b1;				//7z65_4310

  b1 = (wr_data << 4) & 0x40;	//z2zz_zzzz
  b0 = b0 | b1;				//7265_4310

  write_data = b0;

#ifndef SIMU
  outbyte(PCI_dev_po->uioaddr + 0x00 + gr_num, write_data);
#endif

}

/*****************************************************************/
// �|�[�g�ɒ��ڏo�͂���B
/*****************************************************************/
void PoWrite_port( int wr_addr, BYTE wr_data)
{
  outbyte(PCI_dev_po->uioaddr + wr_addr, wr_data);

  return;
}

