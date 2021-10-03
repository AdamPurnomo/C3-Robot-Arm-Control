#include "./PO_128LH.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

int main()
{
  int i;

  ThreadCtl(_NTO_TCTL_IO,0);

  //�{�[�h�̏�����
  if(PoInit() == FALSE) {
    printf("PoInit ERROR!\n");
    return 1;
  }else printf("PoInit OK!\n");

  printf("Po output and input\n");
  for(i=0;i<4;i++) {

    PoWrite_byte( 0x00, 0b00000000); //�T�[�{�I�t
    delay(1000);

    PoWrite_byte( 0x00, 0b01111111); //�T�[�{�I��
    delay(1000);
  }

  PoFin();

  return 0;
}
