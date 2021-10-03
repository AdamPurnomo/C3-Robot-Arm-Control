// INtime�pCNT�{�[�h�iCNT32-8M�j����p�v���O����
#include "CNT32_8M.h"

// �{�[�h�̏��
#define DEVNAME "CNT32-8M"
#define VEN_ID  0x1221  // Vender ID
#define DEV_ID  0xC105  // �f�o�C�XID
#define MAXCH   8       // �{�[�h�̍ő�`�����l����

// �O���[�o���ϐ��錾
static struct PCI_dev_t *PCI_dev_cnt;

/*****************************************************************/
// �������֐�
/*****************************************************************/
BOOL CntInit()
{
  int j;
  unsigned long	cntdata;

  PCI_dev_cnt = (struct PCI_dev_t *)calloc(1,sizeof(struct PCI_dev_t));
  // PCI�f�o�C�X�̏����ݒ�
  if(PCI_Init_new(0, VEN_ID, DEV_ID, DEVNAME,PCI_dev_cnt)){
    return FALSE;
  }

  // ���C���Ŏg��IO�A�h���X�̐ݒ�
  PCI_dev_cnt->uioaddr = PCI_dev_cnt->ioaddr[0];
  //		printf("Main IO ADDRESS is %04x.\n", PCI_dev_cnt[i]->uioaddr);

  // ����ݒ�
  // �ʑ����p���X�A�S���{�A�񓯊��N���A�A�J�E���g�A�b�v�A��v���o����
  // �J�E���^�̑I���@���̓p���X�L��
  // �O�����b�`�̃N���A�Ȃ�
  // ���荞�݃}�X�N�ݒ�@���荞�݂������Ȃ�
  // timer off

  // ----- Initialize -----
  // ���샂�[�h���u�t�B���^0.1us�ACW=UP�A2�����́A�����N���A�A4���{�v�ɐݒ肷��
  for(j=0;j<MAXCH;j++) {
    outhword(PCI_dev_cnt->uioaddr + 0x08, 0x0008+j);
    outword(PCI_dev_cnt->uioaddr + 0x0c, 0x00001022);	// ����
    //	outword(PCI_dev_cnt[i]->uioaddr + 0x0c, 0x00001062);	// TTL
  }

  // �v���Z�b�g���W�X�^�ɍő�l�̔�����ݒ肷��
  for(j=0;j<MAXCH;j++) {
    outhword(PCI_dev_cnt->uioaddr + 0x08, 0x0010+j);
    outword(PCI_dev_cnt->uioaddr + 0x0c, 0x80000000);
  }

  // Z�����͂𖳌��ɐݒ肷��
  for(j=0;j<MAXCH;j++) {
    outhword(PCI_dev_cnt->uioaddr + 0x08, 0x0030+j);
    outword(PCI_dev_cnt->uioaddr + 0x0c, 0x00000001);
  }

  // �v���Z�b�g�f�[�^��SCh�Ƀ��[�h����
  outhword(PCI_dev_cnt->uioaddr + 0x08, 0x003c);
  outword(PCI_dev_cnt->uioaddr + 0x0c, 0x000000ff);

  // �J�E���^��SCh�X�^�[�g����
  outhword(PCI_dev_cnt->uioaddr + 0x04, 0x00FF);

  // //----- Test Pulse Output -----
   outhword(PCI_dev_cnt->uioaddr + 0x08, 0x003e);	// �e�X�g�p���X�ݒ�R�}���h(���g�p�̏ꍇ�͐ݒ�s�v)
   outword(PCI_dev_cnt->uioaddr + 0x0c, 0x0000);	// �e�X�g�p���X������ɏo��(���g�p�̏ꍇ�͐ݒ�s�v)

  // ----- Read Data -----
  // �J�E���g�l��S�`���l�����b�`����
  outhword(PCI_dev_cnt->uioaddr + 0x02, 0x00ff);

  for(j=0;j<MAXCH;j++) {
    outhword(PCI_dev_cnt->uioaddr + 0x00, j);	// �J�E���g�l�̓ǂݏo��Ch���w��
    cntdata = inword(PCI_dev_cnt->uioaddr + 0x00);	// �J�E���g�l��ǂݏo��(DWord�A�N�Z�X�ȊO�s��)

    //	outhword(PCI_dev_cnt[i]->uioaddr + 0x08, 0x003f);	// �X�e�[�^�X���[�h�R�}���h�w��
    //	stsdata = inword(PCI_dev_cnt[i]->uioaddr + 0x0c);	// �X�e�[�^�X��ǂݏo��

    //	printf("CountData  StatusData\n");		// �J�E���g�l�A�X�e�[�^�X�\��
    //	printf("%08ld      %02x\n", cntdata, stsdata);
  }
  return TRUE;
}


/*****************************************************************/
// �J�E���^�{�[�h1�����̒l�̓ǂݏo��
/*****************************************************************/
/*
void CntRead(int dev_num, DWORD cnt[])
{
	int i;
	static long ticks = 0;
//	unsigned long	stsdata;


	outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x02, 0x00ff);	// �S�`���l�����b�`

	for(i=0;i<MAXCH;i++) {
		outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x00, i);	// �J�E���g�l�̓ǂݏo��Ch���w��
		cnt[i] = inword(PCI_dev_cnt[dev_num]->uioaddr + 0x00);	// �J�E���g�l��ǂݏo��(DWord�A�N�Z�X�ȊO�s��)

	//	outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x08, 0x003f);	// �X�e�[�^�X���[�h�R�}���h�w��
	//	stsdata = inword(PCI_dev_cnt[dev_num]->uioaddr + 0x0c);	// �X�e�[�^�X��ǂݏo��

	//	printf("CountData  StatusData\n");		// �J�E���g�l�A�X�e�[�^�X�\��
	//	printf("%08ld      %02x\n", cnt[i], stsdata);
	}

	return;
}
*/

void CntRead_c3kai(DWORD cnt[])
{
  int i;
  DWORD	buf;

  outhword(PCI_dev_cnt->uioaddr + 0x02, 0x00ff);	// �S�`���l�����b�`

  for(i=0;i<MAXCH;i++) {
    if(i == 3)  outhword(PCI_dev_cnt->uioaddr + 0x00, 7);	// �J�E���g�l�̓ǂݏo��Ch���w��
    else        outhword(PCI_dev_cnt->uioaddr + 0x00, i);	// �J�E���g�l�̓ǂݏo��Ch���w�� 
    cnt[i] = inword(PCI_dev_cnt->uioaddr + 0x00);	// �J�E���g�l��ǂݏo��(DWord�A�N�Z�X�ȊO�s��)

    //	outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x08, 0x003f);	// �X�e�[�^�X���[�h�R�}���h�w��
    //	stsdata = inword(PCI_dev_cnt[dev_num]->uioaddr + 0x0c);	// �X�e�[�^�X��ǂݏo��

    //	printf("CountData  StatusData\n");		// �J�E���g�l�A�X�e�[�^�X�\��
    //	printf("%08ld      %02x\n", cnt[i], stsdata);
  }

  //6�Ԗڂ̒l��2�Ԗڂɓ���āA2�Ԗڈȍ~�����̔ԍ��ɂ��炷 nan
  buf=cnt[6];

  cnt[6]=cnt[5];
  cnt[5]=cnt[4];
  cnt[4]=cnt[3];
  cnt[3]=cnt[2];
  cnt[2]=buf;

  return;
}
/*****************************************************************/
// �J�E���^�{�[�h1�����̒l�̏�������
/*****************************************************************/
/*
void CntWrite(int dev_num, DWORD cnt[])
{
	int i;

	// �J�E���^��SCh�X�g�b�v����
	outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x04, 0x0000);

	// �v���Z�b�g���W�X�^�ɒl��ݒ肷��
	for(i=0;i<MAXCH;i++) {
		outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x08, 0x0010+i);
		outword(PCI_dev_cnt[dev_num]->uioaddr + 0x0c, cnt[i]);
	}

	// �v���Z�b�g�f�[�^��SCh�Ƀ��[�h����
	outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x08, 0x003c);
	outword(PCI_dev_cnt[dev_num]->uioaddr + 0x0c, 0x000000ff);

	// �J�E���^��SCh�X�^�[�g����
	outhword(PCI_dev_cnt[dev_num]->uioaddr + 0x04, 0x00FF);

	return;
}
*/

void CntWrite_c3kai(DWORD cnt[])
{
  int i;
  DWORD	buf;

  //2�Ԗڂ̒l��6�Ԗڂɓ���āA2�Ԗڈȍ~��O�̔ԍ��ɂ��炷 nan
  buf=cnt[2];

  cnt[2]=cnt[3];
  cnt[3]=cnt[4];
  cnt[4]=cnt[5];
  cnt[5]=cnt[6];
  cnt[6]=buf;

  // �J�E���^��SCh�X�g�b�v����
  outhword(PCI_dev_cnt->uioaddr + 0x04, 0x0000);

  // �v���Z�b�g���W�X�^�ɒl��ݒ肷��
  for(i=0;i<MAXCH;i++) {
    outhword(PCI_dev_cnt->uioaddr + 0x08, 0x0010+i);
    outword(PCI_dev_cnt->uioaddr + 0x0c, cnt[i]);
  }

  // �v���Z�b�g�f�[�^��SCh�Ƀ��[�h����
  outhword(PCI_dev_cnt->uioaddr + 0x08, 0x003c);
  outword(PCI_dev_cnt->uioaddr + 0x0c, 0x000000ff);

  // �J�E���^��SCh�X�^�[�g����
  outhword(PCI_dev_cnt->uioaddr + 0x04, 0x00FF);

  return;
}


/*****************************************************************/
// �I���֐�
/*****************************************************************/
void CntFin()
{
  int i;

  for(i=0;i<4;i++) {
    outbyte(PCI_dev_cnt->ioaddr[i] + 0x06, 0x03);
    outbyte(PCI_dev_cnt->ioaddr[i] + 0x16, 0x03);
  }
  free((void *)PCI_dev_cnt);

  //	printf("%s finished!\n", DEVNAME);
  return ;
}


/*****************************************************************/
// �C�ӂ̃J�E���^�l���Z�b�g����B
/*****************************************************************/
void CntSet(int ch, DWORD cnt)
{
	// 0,2,4,6��
	if(ch%2 == 0) {
		outbyte(PCI_dev_cnt->ioaddr[ch/2] + 0x00, cnt);
		outbyte(PCI_dev_cnt->ioaddr[ch/2] + 0x01, cnt>>8);
		outbyte(PCI_dev_cnt->ioaddr[ch/2] + 0x02, cnt>>16);
	}else {
		outbyte(PCI_dev_cnt->ioaddr[ch/2] + 0x10, cnt);
		outbyte(PCI_dev_cnt->ioaddr[ch/2] + 0x11, cnt>>8);
		outbyte(PCI_dev_cnt->ioaddr[ch/2] + 0x12, cnt>>16);
	}
}

