#include "DIO_6464T2.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

int main()
{
  int i;
  BYTE  bk;
  BYTE  srdy;
  BYTE  alm;
  BYTE  tgon;

  ThreadCtl(_NTO_TCTL_IO,0);

  //�{�[�h�̏�����
  if(DioInit() == FALSE) {
    printf("DioInit ERROR!\n");
    return 1;
  }
  else printf("DioInit OK!\n");

  printf("Dio output and input\n");
  for(i=0;i<500;i++) {
    //DioWrite_byte(0x06, 0x00);				// SEL���I����Ԃɂ���

    // /BK��ǂݏo��
    // 0xC0�FJ1�`J6�u���[�L�I�t, 0xFF�F�u���[�L�I��
    bk = ~DioRead_byte(0x03);

    // /S-RDY��ǂݏo��
    // S-RDY : �T�[�{�p�b�N���T�[�{�I��(/S-ON)�M������t�\���ǂ����ǂ�
    // 0xC0�F�T�[�{���f�B(6��)�C0x80�F�T�[�{���f�B(7��)
    srdy = ~DioRead_byte(0x00);

    // ALM��ǂݏo��
    // ALM : �ُ�����o�����Ƃ��ɁC�I�t(�J)�ɂ���
    // 0xFE�F�ُ�Ȃ� 0xFF�F�A���[������
    alm = ~DioRead_byte(0x01);

    // TGON : �T�[�{���[�^���ݒ�l�ȏ�̉�]���x�ɂȂ����ꍇ�I��(��)����
    // 0xFF�FJ1�`J6��~���A0xC0�FJ1�`J6��]��
    tgon = ~DioRead_byte(0x02);

    printf("%d  /BK: %x, /SRDY: %x, /ALM: %x, /TGON: %x\n",i,bk,srdy,alm,tgon);

    delay(100);
  }

  DioFin();

  return 0;
}
