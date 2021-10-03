#ifndef _ARC_PCI_H
#define _ARC_PCI_H

extern void GetPosition(double *cur_pos);
extern void AGetCmdVolt(double *cmd_volt);  //elase after check
extern void AGetPulse(double *cur_pulse);   //elase after check
extern void SetTorq(double *torq);
extern void NoTorq(void);
extern void BrakeOFF(int joint);
extern void AllBrakeOFF(void);
extern void NoSpeed(void);
extern int  RecData(void);
extern int  iSend_C(void);

extern int arcInit(void);
extern void arcnet_start(void);
extern void arcFin(void);

#endif /* _ARCPCI_H */




