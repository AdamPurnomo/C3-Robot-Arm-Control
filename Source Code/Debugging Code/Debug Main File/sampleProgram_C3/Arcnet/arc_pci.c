/*******************************************************

	     ARC Net Driver for PA-10

********************************************************/

#include "./DeviceIF/DeviceIF.h"
#include "arc_pci.h"
#include "arc_pci_cfg.h"


int RecData(void)
{
  return DeviceUpdate();
}

void GetPosition(double *cur_pos)
{
  D_GetPosition(cur_pos);
}

void AGetCmdVolt(double *cmd_volt)
{
  GetCmdVolt(cmd_volt);
}

void AGetPulse(double *cur_pulse)
{
  GetPulse(cur_pulse);
}

void SetTorq(double *torq)
{
  D_SetTorq(torq);
}

void NoTorq(void)
{
  D_NoTorq();
}

void BrakeOFF(int joint)
{
}

void AllBrakeOFF(void)
{
}


int iSend_C(void)
{
  return OK;
}


int arcInit(void)
{
  switch(CtrlMode){
  case operation:
    DeviceInit();
    break;
  case simulation:
    break;

  default:
    break;
  }
  return OK;
}

void arcnet_start(void)
{
  switch(CtrlMode){
  case operation:
    DeviceStart();
    break;
  case simulation:
    break;
  default:
    break;
  }
}

void arcFin(void)
{
  switch(CtrlMode){
  case operation:
    DeviceFin();
    break;
  case simulation:
    break;
 
  default:
    break;
  }
}
