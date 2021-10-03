#include "./PI_128LH.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

int main()
{
  int i;
  BYTE Bk;

  ThreadCtl(_NTO_TCTL_IO,0);

  //�{�[�h�̏�����
  if(PiInit() == FALSE) {
    printf("PiInit ERROR!\n");
    return 1;
  }
  else printf("PiInit OK!\n");

  printf("Pi output and input\n");
  for(i=0;i<10;i++) {
    Bk = PiRead_byte(0x07);//�ً}��~�X�C�b�`
    printf("emergency switch: %x \n",Bk);

    delay(1000);
  }

  PiFin();

  return 0;
}
