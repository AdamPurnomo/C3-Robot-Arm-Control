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
  for(i=0;i<40;i++) {

    printf("Servo:OFF(0x00), Control:Torqu(0x00)\n");
    PoWrite_byte( 0x00, 0x00); //�T�[�{�I�t
    PoWrite_byte( 0x01, 0x00); //�g���N����
    delay(1000);

    printf("Servo:ON(0x80),  Control:Speed(0x7F)\n");
    PoWrite_byte( 0x00, 0x7F); //�T�[�{�I��
    PoWrite_byte( 0x01, 0x7F); //���x����
    delay(1000);
  }

  PoFin();

  return 0;
}
