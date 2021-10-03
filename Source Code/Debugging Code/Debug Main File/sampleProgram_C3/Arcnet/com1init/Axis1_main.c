
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>
#include "REX_PCI60D.h"
#include "pcidev.h"

int main()
{
  int           i,j;
  BYTE          Sen;
  char          rxdata[10];             // unsigned char ���� char�ɕύX 20181020 yoshinaga
  const int     max_rx_bytes = 8;

  ThreadCtl(_NTO_TCTL_IO, 0);

  printf("\n");
  printf("com1init\n");
  Com1Init();

  if (Com1Init() == FALSE) {
    printf("Com1Init ERROR!\n");
  }
  else printf("Com1Init OK!");

  for(j=0;j<max_rx_bytes+1;j++) {     // �ϐ��̏�����
    *(rxdata+j) = 0x00;
  }

  DioWrite_byte(0x06, 0x00);          // SEL���I����Ԃɂ���
  DioWrite_byte(0x04, 0xFF);          // SEN��S��Low�ɂ��� ���_��

  Sen = 0x01;                         // SEN�͂P�䂸��High�ɂ���
  DioWrite_byte(0x04, ~Sen);          // SEN���o�͂��� ���_��
  delay(100);

  Com1FifoClr();
  DioWrite_byte(0x06, ~i);            // SEL��0x0�`0x6 ���_��

  err |= Com1Rx(rxdata, max_rx_bytes);
  DioWrite_byte(0x06, 0x00);          // SEL���I����Ԃɂ���
  // ��M�f�[�^���`�F�b�N����
  if(((*(rxdata+0)!=0x50) || (*(rxdata+7)!=0x0D))
      || ((*(rxdata+1)!=0x2B) && (*(rxdata+1)!=0x2D))) {	// '0x50'�C'0x0D'�ȂǃA�X�L�[�R�[�h
    err = 1;
    printf("RxData error!\n");
  }
  for(j=2; j<7; j++) {
    if( (*(rxdata+j) < 0x30) || (*(rxdata+j) > 0x39) ) {	// '0x30'�C'0x39'�ȂǃA�X�L�[�R�[�h
      err = 1;
      printf("RxData error!\n");
    }
  }

  AbsPos[i] = atoi(rxdata+1);      // AbsPos0[] : ����]��

  // ����]�ʒu���`�F�b�N����
  if( (AbsPos[i] < -38) || (AbsPos[i] > 38) ) {
    err = 1;
    printf("AbsPos error!\n");
  }

  // struct PCI_dev_t buff;
  // PCI_Init_new(0, 0x13a8, 0x152, 0,  &buff);
  // printf("buff.ven_id:%lx\n", buff.ven_id);
  // printf("buff.dev_id:%lx\n", buff.dev_id);
  // printf("buff.uioaddr:%lx\n", buff.uioaddr);
  // printf("buff.rev_id:%x\n", buff.rev_id);
  // printf("buff.ioaddr[0]:%lx\n", buff.ioaddr[0]);
  // printf("buff.ioaddr[1]:%lx\n", buff.ioaddr[1]);
  // printf("buff.ioaddr[2]:%lx\n", buff.ioaddr[2]);
  // printf("buff.ioaddr[3]:%lx\n", buff.ioaddr[3]);
  // printf("buff.ioaddr[4]:%lx\n", buff.ioaddr[4]);
  // printf("buff.ioaddr[5]:%lx\n", buff.ioaddr[5]);

  return 0;
}
