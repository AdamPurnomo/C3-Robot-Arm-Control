//#include "DIO_6464T2.h"
#include <stdio.h>
#include <stdlib.h>
#include "pcidev.h"

#define VEN_ID	0x1221	// Vender ID
#define DEV_ID	0xa1d2	// デバイスID
#define DEVNAME "DIO-6464T2"

//extern PCI_dev_t *PCI_Init_new(int dev_num, WORD ven_id, WORD dev_id, char *dev_name);

int main()
{
  struct PCI_dev_t *PCI_dev_tmp;
  PCI_dev_tmp = (struct PCI_dev_t*)calloc(1,sizeof(struct PCI_dev_t));

  if(PCI_Init_new(0, VEN_ID, DEV_ID, DEVNAME,PCI_dev_tmp) == 0){
    printf("BAR[0] = %x\n",PCI_dev_tmp->ioaddr[0]);
    printf("success\n");
  }
  //DioInit();
  return 0;
}
