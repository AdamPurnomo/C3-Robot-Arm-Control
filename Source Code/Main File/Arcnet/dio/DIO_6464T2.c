// INtime�p��≏DIO�{�[�h�iDIO-6464T2�j����p�v���O����
#include "DIO_6464T2.h"

// �{�[�h�̏��
#define DEVNAME "DIO-6464T2"
#define VEN_ID  0x1221  // Vender ID
#define DEV_ID  0xB122  // �f�o�C�XID
#define MAXINCH		64	// IN���̃`�����l����
#define MAXOUTCH	64	// OUT���̃`�����l����

// �O���[�o���ϐ��錾
static struct PCI_dev_t *PCI_dev_dio;

// ���̃t�@�C�����݂̂Ŏg�p����v���g�^�C�v�錾
// PCI_dev_t *PCI_Init_DIO(int dev_num);

/*****************************************************************/
// �������֐�
/*****************************************************************/

BOOL DioInit()
{
  int j;

  // PCI�f�o�C�X�������̍��}
  //	printf("Searching %s....\n", DEVNAME);

  PCI_dev_dio = (struct PCI_dev_t *)calloc(1,sizeof(struct PCI_dev_t));
  // PCI�f�o�C�X�̏����ݒ�
  if(PCI_Init_new(0,VEN_ID, DEV_ID, DEVNAME,PCI_dev_dio)){
    return FALSE;
  }
  // ���C���Ŏg��IO�A�h���X�̐ݒ�
  PCI_dev_dio->uioaddr = PCI_dev_dio->ioaddr[0];
  //		printf("Main IO ADDRESS is %04x.\n", PCI_dev_dio->uioaddr);

  // ����ݒ�
  // �f�W�^���t�B���^�i���g�p�j
  // ���荞�݃}�X�N�i���g�p�j
  // ���荞�݃G�b�W�I���i���g�p�j
  // �C���^�[�o���^�C�}�i���g�p�j

  // �o�͉�H�̓I�[�v���R���N�^�ŕ��_��
  // ON��Low�ɁAOFF��pullup��R�ɂ��High�ɂȂ�
  // ���ׂ�ON(Low)�ɂ��Ă���
  for(j=0;j<8;j++) {
    outbyte(PCI_dev_dio->uioaddr + 0x08+j, 0xFF);
  }

  // �u���[�L�M����OFF�ɂ��Ă���
  outbyte(PCI_dev_dio->uioaddr + 0x08, 0x00);

  return TRUE;
}


/*****************************************************************/
// �I���֐�
/*****************************************************************/
void DioFin()
{
  int j;

  // �o�͂͂��ׂ�ON(Low)�ɂ��Ă���
  for(j=0;j<8;j++)
    outbyte(PCI_dev_dio->uioaddr + 0x08+j, 0xFF);

  // �u���[�L�M����OFF�ɂ��Ă���
  outbyte(PCI_dev_dio->uioaddr + 0x08, 0x00);

  // �������[�̉��
  free((void *)PCI_dev_dio);
  //	printf("%s finished.\n", DEVNAME);

  return ;
}
/**************************************************/
/* DIO -> ���̓|�[�g�@"BaseAddress + 0x00"        */
/*        �o�̓|�[�g  "BaseAddress + 0x08" kamei  */
/**************************************************/

/*****************************************************************/
// �w��̂P�r�b�g��ǂݍ���(���_��)
// dev_num;�{�[�hNO
// ch_num;���͂������`�����l���ԍ�(0-63)
/*****************************************************************/
BYTE DioRead_bit(unsigned int ch_num)
{
  BYTE port;
  BYTE onoff;

  port = inbyte(PCI_dev_dio->uioaddr + 0x00 + (ch_num>>3));
  onoff = port >> (ch_num - (ch_num&0xF8));
  //	printf("DioRead_bit : port=%d, onoff=%d\n", port, onoff);
  return onoff;
}

/*****************************************************************/
// �w��̂P�o�C�g��ǂݍ���
// dev_num;�{�[�hNO
// gr_num;���͂������O���[�v�ԍ�(0-7)
/*****************************************************************/
BYTE DioRead_byte(unsigned int gr_num)
{
  BYTE port;

  port = inbyte(PCI_dev_dio->uioaddr + 0x00 + gr_num);
  return port;
}

/*****************************************************************/
// �{�[�hBNO��OUTn�ɏ����o��
// dev_num;�{�[�hNO
// outsignal;�o�͂�����port�ԍ�
// outch;�e�`�����l����ON�i1�j�AOFF(0)�̔z��i64�`�����l���j
/*****************************************************************/
void DioWrite_all(BYTE outch[])
{
  // 	int i,j;
  int i;
  const int p_num = 8;	// 8�̃|�[�g������
  BYTE port[8];

  for(i=0;i<p_num;i++) {
    port[i] = 0x00;
    port[i] = outch[0+i*8]*1 + outch[1+i*8]*(2) + outch[2+i*8]*(4) 
      + outch[3+i*8]*(8) + outch[4+i*8]*(16) + outch[5+i*8]*(32)
      + outch[6+i*8]*(64) + outch[7+i*8]*(128);
  }

  for(i=0;i<p_num;i++) {
#ifndef SIMU
    outbyte(PCI_dev_dio->uioaddr + 0x08 + i, port[i]);
#endif
  }

}

/*****************************************************************/
// �{�[�hBNO��OUTn�ɏ����o��
// dev_num;�{�[�hNO
// gr_num;�o�͂������O���[�v�ԍ�(0-7)
// wr_data;�W�r�b�g��1(High�j�A0(Low)�̔z��
/*****************************************************************/
void DioWrite_byte(unsigned int gr_num, BYTE wr_data)
{

#ifndef SIMU
  outbyte(PCI_dev_dio->uioaddr + 0x08 + gr_num, wr_data);
#endif

}

/*****************************************************************/
// �|�[�g�ɒ��ڏo�͂���B
/*****************************************************************/
void DioWrite_port(int wr_addr, BYTE wr_data)
{
  outbyte(PCI_dev_dio->uioaddr + wr_addr, wr_data);

  return;
}


