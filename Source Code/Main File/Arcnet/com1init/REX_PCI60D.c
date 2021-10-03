// qnx�pREX-PCI60D(RS232C�V���A���ʐM)����p�v���O����
#include "REX_PCI60D.h"

#ifndef PortAddress
#define PortAddress (0x8000)
#endif
/*---portaddress kobari*/

#define Delay() (void)inbyte(PortAddress+0x07)//koko
;
// �֐��̃v���g�^�C�v�錾
//BOOL Com1Init();
//BOOL Com1Tx(BYTE* txdata, int txlength);
//BOOL Com1Rx(BYTE* rxdata, int rxlength);
//void Com1FifoClr();

/*****************************************************************/
// �������֐�
/*****************************************************************/

BOOL Com1Init()
{
  // COM1(�V���A���|�[�g)�̏�����
  // RS-232C�̒[�q�d���͕��_���ALSB�t�@�[�X�g
  WORD address;

  address = mmap_device_io(1,PortAddress);
  outbyte(address+ 0x003, 0x80);				// LCR���W�X�^DLA�r�b�g//koko
  printf("LCR = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x003)));//koko
  Delay();

  // �ʐM���x9600bps
  outbyte(mmap_device_io(1,PortAddress + 0x000), 0x00);				// THR/RBR(Divisor L)���W�X�^//koko
  //outbyte(PortAddress+0x000, 0x0C);				// THR/RBR(Divisor L)���W�X�^//koko
  printf("THR(Divisor L) = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x000)));//koko
  Delay();

  outbyte(mmap_device_io(1,PortAddress + 0x001), 0x00);				// IER(Divisor H)���W�X�^//koko
  printf("IER(Divisor H) = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x001)));//koko
  Delay();

  // outbyte(PortAddress+0x003, 0x13);			// LCR���W�X�^DLA�r�b�g Parity�Ȃ�8bit//koko
  outbyte(mmap_device_io(1,PortAddress + 0x003), 0x1A);				// LCR���W�X�^DLA�r�b�g Parity����7bit//koko
  printf("LCR = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x003)));//koko
  Delay();

  outbyte(mmap_device_io(1,PortAddress + 0x001), 0x00);				// IER���W�X�^ ���荞�݋֎~//koko
  printf("IER = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x001)));//koko
  Delay();

  outbyte(mmap_device_io(1,PortAddress + 0x004), 0x00);				// MCR���W�X�^//koko
  printf("MCR = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x004)));//koko
  Delay();

  outbyte(mmap_device_io(1,PortAddress + 0x002), 0x87);				// FCR(IIR)���W�X�^ W only//koko
  printf("IIR(FCR) = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x002)));//koko
  Delay();

  outbyte(mmap_device_io(1,PortAddress + 0x004), 0x03);				// MCR���W�X�^ RTS,DTR//koko
  printf("MCR = %02x\n", inbyte(mmap_device_io(1,PortAddress + 0x004)));//koko
  Delay();

  //	inbyte(PortAddress+0x005);					// LSR���W�X�^ �X�e�[�^�X//koko

  return TRUE;
}

BOOL Com1Tx(BYTE* txdata, int txlength)
{
  int i;
  printf("%x\n", (unsigned int)txdata);
  i=0;
  while(i < txlength) {
    if ((inbyte(mmap_device_io(1,PortAddress + 0x006)) & 0x10)) {	// MSR���W�X�^CTS�r�b�g//koko
      outbyte(mmap_device_io(1,PortAddress + 0x000), txdata[i]);//koko
      i++;
    }
    else {
      ;
    }
    Delay();						// MSR���W�X�^CTS�r�b�g�҂�
    // printf("i:%i\n", i);
    // printf("status %x\n", inbyte(mmap_device_io(1, PortAddress + 0x006)));
  }

  printf("%x\n", (unsigned int)txdata);
  return TRUE;
}

BOOL Com1Rx(BYTE* rxdata, int rxlength)
{
  int		i, j;
  BOOL	err;
  BYTE	rxstatus;

  err       = 0;
  i         = 0;
  j         = 0;
  rxstatus  = 0;

  while(i < rxlength) {
    rxstatus = inbyte(mmap_device_io(1,PortAddress + 0x05));		// LSR���W�X�^//koko
    if(rxstatus & 0x08) {				// FramingErr�r�b�g
      *(rxdata + i) = inbyte(mmap_device_io(1,PortAddress + 0x00));	// RBR���W�X�^//koko
      printf("FramingErr at byte No.%d\n", i);
      err = 1;
    }
    else if(rxstatus & 0x04) {			// ParityErr�r�b�g
      *(rxdata + i) = inbyte(mmap_device_io(1,PortAddress + 0x00));	// RBR���W�X�^//koko
      printf("ParityErr at byte No.%d\n", i);
      err = 1;
    }
    else if(rxstatus & 0x02) {			// FifoOverflow�r�b�g
      *(rxdata + i) = inbyte(mmap_device_io(1,PortAddress + 0x00));	// RBR���W�X�^//koko
      printf("FifoOverflow at byte No.%d\n", i);
      err = 1;
    }
    else if(rxstatus & 0x01) {			// DataReady�r�b�g
      *(rxdata + i) = inbyte(mmap_device_io(1,PortAddress + 0x00));	// RBR���W�X�^//koko
      j = 0;
      i++;
    }
    else if((i>0) && (j>200)) {			// �^�C���A�E�g
      i = rxlength;
    }
    else if((i==0) && (j>1000)) {		// �^�C���A�E�g
      printf("TimeoutErr at byte No.%d0\n", i);
      printf("%x\n",rxstatus);
      printf("%x\n",PortAddress);
      err = 1;
    }
    else {
      j++;
    }
    Delay();						// 1ms�҂�
    if(err) break;
  }
  printf("rxstatus %x\n", (unsigned int)rxstatus);
  printf("rxdata %x\n",(unsigned int)rxdata);
  return err;
}

void Com1FifoClr()
{
  outbyte(mmap_device_io(1,PortAddress + 0x002), 0x87);			// FCR���W�X�^ FIFO�N���A//koko
  Delay();
  inbyte(mmap_device_io(1,PortAddress + 0x005));				// LSR���W�X�^ �X�e�[�^�X//koko
  inbyte(mmap_device_io(1,PortAddress + 0x000));				// RBR���W�X�^ ��M�f�[�^//koko
}


