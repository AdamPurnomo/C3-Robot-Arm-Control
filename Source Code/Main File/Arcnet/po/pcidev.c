#include "./pcidev.h"

#include <hw/pci.h>
#include <hw/pci_devices.h>
#include <hw/inout.h> 
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/neutrino.h>
#include <string.h>

// PCIデバイスの検出処理
int PCI_Init_new(int dev_num, WORD ven_id, WORD dev_id, char *dev_name, struct PCI_dev_t *buff)
{
  int   i;

  int   pidx;
  int   phdl;
  void* hdl;
  void* retval;
  struct pci_dev_info   inf;

  if(buff == NULL){
    printf("ERROR Can't allocate enough memory\n");
    return 1;
  }

  phdl = pci_attach( 0 );
  if( phdl == -1 ) {
    fprintf( stderr, "Unable to initialize PCI\n" );
    return 1;
  }

  memset( &inf, 0, sizeof( inf ) );
  pidx = 0;
  inf.VendorId = ven_id;
  inf.DeviceId = dev_id;

  hdl = pci_attach_device( NULL, 0, pidx, &inf );
  if( hdl == NULL ) {
    fprintf( stderr, "Unable to locate adapter\n" );
  }

  retval = pci_attach_device( hdl, PCI_INIT_ALL, pidx, &inf );
  if( retval == NULL ) {
    fprintf( stderr, "Unable allocate resources\n" );
  }

  for(i=0;i<6;i++)
    if(inf.PciBaseAddress[i] & 0x1)
      buff->ioaddr[i] = inf.PciBaseAddress[i] & 0xfffc;

  //for(i=0;i<6;i++) printf("BAR[%d]: %x\n",i,(int)inf.PciBaseAddress[i]);

  pci_detach( phdl );

  buff->ven_id = (WORD)ven_id;
  buff->dev_id = (WORD)dev_id;
  buff->rev_id = 0;

  return 0;
}

BYTE inbyte (WORD port)
{
  return (BYTE)in8(port);
}

WORD inhword (WORD port)
{
  return (WORD)in16(port);
}

DWORD inword (WORD port)
{
  return (DWORD)in32(port);
}

void outbyte (WORD port, BYTE value)
{
  out8(port,value);
}

void outhword (WORD port, WORD value)
{
  out16(port,value);
}

void outword (WORD port, DWORD value)
{
  out32(port,value);
}

