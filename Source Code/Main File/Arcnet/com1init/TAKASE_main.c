
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>
#include "pcidev.h"
#include "REX_PCI60D.h"

int main()
{
  int j;
  unsigned char rxdata[10];
  unsigned char txdata[10];
  const int     max_rx_bytes = 8;

  for(j=0;j<max_rx_bytes+1;j++) {
    *(rxdata+j) = 0x00;
  }

  printf("main!!!!!!!!!!!!\n");

  ThreadCtl(_NTO_TCTL_IO, 0);

  printf("\n");
  printf("com1init!!!!!!!!!!!!!!!!\n");
  Com1Init();

  if (Com1Init() == FALSE) {
    printf("Com1Init ERROR!\n");
  }//else SendMess("Com1Init OK!\n");
  else printf("Com1Init OK!");

  printf("\n");
  printf("com1Tx!!!!!!!!!!!!!!!!!!!!\n");
  Com1Tx(txdata,8);


  printf("\n");
  printf("com1Rx!!!!!!!!!!!!!!!!!!!\n");
  Com1Rx(rxdata, 8);

  printf("\n");
  printf("PCI_Init_new!!!!!!!!!!!!!!!!!!!!!\n");

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
