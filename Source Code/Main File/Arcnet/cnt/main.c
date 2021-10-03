#include "CNT32_8M.h"
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

#define PI     ((double)3.14159265358979323846)
#define ENC2DEG ((360.0)/0x00010000) // ÇPâÒì]ÅÅ16384ÉpÉãÉXÅ~4í¸î{

// int     PoOffset[]    = {-23505, -41356, -39319, -44537, -39831, -49087, 0};
double  RRatio[]  = {-1.0/80.0, -1.0/80.0, -1.0/70.0, -55.0/71.0/50.0,
                      -55.0/45.0/80.0, 1.0/50.0, -1.0/50.0};
int main()
{
  int i,j;
  double  CurActAgl[7]; // é¿ç€ÇÃà íu
  DWORD   CurCnt[7];

  ThreadCtl(_NTO_TCTL_IO,0);

  if(CntInit() == FALSE) {
    printf("CntInit ERROR!\n");
    return 1;
  }
  else printf("CntInit OK!\n");

  printf("print each servo value\n");
  printf("Raw value | Angle\n");
  for(i=0;i<300;i++) {
    delay(100);
    // åªç›ÇÃÉpÉãÉXà íuÇÃéÊìæ
    CntRead_c3kai(CurCnt);

    // 0x80000000â∫ë óöÇ´âè¡
    for ( j=0; j<8; j++ )
      CurCnt[j] -= 0x80000000;

    for(j=0;j<7;j++) printf(" %d,",(int)CurCnt[j]);
    printf("  |  ");
    for(j=0;j<7;j++) {
      // CurActAgl[j] = ((long)CurCnt[j] - PoOffset[j]) * ENC2RAD*RRatio[j];
      CurActAgl[j] = (long)CurCnt[j] * ENC2DEG*RRatio[j];
      printf(" %f,",CurActAgl[j]);
    }
    printf("\n");
  }

  CntFin();

  return 0;
}
