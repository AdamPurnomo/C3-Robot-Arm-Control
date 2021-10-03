#include "DIO_6464T2.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

int main()
{
  int i;
  BYTE Bk;

  ThreadCtl(_NTO_TCTL_IO,0);

  //�{�[�h�̏�����
  if(DioInit() == FALSE) {
    printf("DioInit ERROR!\n");
    return 1;
  }
  else printf("DioInit OK!\n");

  printf("Dio output and input\n");
  printf("Brake ON 5sec\n");
  for(i=0;i<5;i++) {
    //Brake ON
    DioWrite_byte(0x06, 0xFF);

    // /BK��ǂݏo��
    // 0xC0�FJ1�`J6�u���[�L�I�t, 0xFF�F�u���[�L�I��
    Bk = ~DioRead_byte(0x00);
    printf("BK = %x", Bk);
    printf("\n");

    delay(1000);
  }

  printf("Brake OFF 2sec\n");
  for(i=0;i<2;i++) {
    //Brake ON
    DioWrite_byte(0x06, 0x00);

    // /BK��ǂݏo��
    // 0xC0�FJ1�`J6�u���[�L�I�t, 0xFF�F�u���[�L�I��
    Bk = ~DioRead_byte(0x00);
    printf("BK = %x", Bk);
    printf("\n");

    delay(1000);
  }

  printf("Brake ON 5sec\n");
  for(i=0;i<5;i++) {
    //Brake ON
    DioWrite_byte(0x06, 0xFF);

    // /BK��ǂݏo��
    // 0xC0�FJ1�`J6�u���[�L�I�t, 0xFF�F�u���[�L�I��
    Bk = ~DioRead_byte(0x00);
    printf("BK = %x", Bk);
    printf("\n");

    delay(1000);
  }

  DioFin();

  return 0;
}
