#include "DIO_6464T2.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

int main()
{
  int i;

  ThreadCtl(_NTO_TCTL_IO,0);

  //ƒ{[ƒh‚Ì‰Šú‰»
  if(DioInit() == FALSE) {
    printf("DioInit ERROR!\n");
    return 1;
  }
  else printf("DioInit OK!\n");

  printf("Dio output and input\n");
  for(i=0;i<40;i++) {

    printf("SEN: %x\n",0xFF);
    DioWrite_byte(0x04, 0xFF);				// SEN‚ğLow
    delay(1000);

    printf("SEN: %x\n",0x00);
    DioWrite_byte(0x04, 0x00);				// SEN‚ğHigh
    delay(1000);
  }

  DioFin();

  return 0;
}
