#include "DeviceIF.h"

static int    is_start;

static double is_BK;

static double cur_pos[7];
static double pre_pos[7];

static double cur_vel[7];
static double pre_vel[7];

static double cur_torq[7];
static double pre_torq[7];
static double cmd_torq[7];
static double cmd_volt[7];

const double  max_pos_limit[] = { MAXANG1*DEG2RAD, MAXANG2*DEG2RAD, MAXANG3*DEG2RAD, MAXANG4*DEG2RAD, MAXANG5*DEG2RAD, MAXANG6*DEG2RAD, MAXANG7*DEG2RAD};
const double  min_pos_limit[] = { MINANG1*DEG2RAD, MINANG2*DEG2RAD, MINANG3*DEG2RAD, MINANG4*DEG2RAD, MINANG5*DEG2RAD, MINANG6*DEG2RAD, MINANG7*DEG2RAD};
const double  hard_max_pos_limit[]  = { HARDMAXANG1*DEG2RAD, HARDMAXANG2*DEG2RAD, HARDMAXANG3*DEG2RAD, HARDMAXANG4*DEG2RAD, HARDMAXANG5*DEG2RAD, HARDMAXANG6*DEG2RAD, HARDMAXANG7*DEG2RAD};
const double  hard_min_pos_limit[]  = { HARDMINANG1*DEG2RAD, HARDMINANG2*DEG2RAD, HARDMINANG3*DEG2RAD, HARDMINANG4*DEG2RAD, HARDMINANG5*DEG2RAD, HARDMINANG6*DEG2RAD, HARDMINANG7*DEG2RAD};

const double  vel_limit[]           = { MAXSPEED1*DEG2RAD, MAXSPEED2*DEG2RAD, MAXSPEED3*DEG2RAD, MAXSPEED4*DEG2RAD, MAXSPEED5*DEG2RAD, MAXSPEED6*DEG2RAD, MAXSPEED7*DEG2RAD};
const double  hard_vel_limit[]      = { HARDMAXSPEED1*DEG2RAD, HARDMAXSPEED2*DEG2RAD, HARDMAXSPEED3*DEG2RAD, HARDMAXSPEED4*DEG2RAD, HARDMAXSPEED5*DEG2RAD, HARDMAXSPEED6*DEG2RAD, HARDMAXSPEED7*DEG2RAD};

const double  torq_limit[]          = { MAX_TORQUE1, MAX_TORQUE2, MAX_TORQUE3, MAX_TORQUE4, MAX_TORQUE5, MAX_TORQUE6, MAX_TORQUE7};
const double  hard_torq_limit[]     = { HARD_MAX_TORQUE1, HARD_MAX_TORQUE2, HARD_MAX_TORQUE3, HARD_MAX_TORQUE4, HARD_MAX_TORQUE5, HARD_MAX_TORQUE6, HARD_MAX_TORQUE7};

const double  RRatio[]              = { -1.0/80.0,      -1.0/80.0,      -1.0/51.0,
                                        -1.0/70.0,      -55.0/71.0/50.0,-55.0/45.0/80.0, 1.0/50.0};
//const int     PoOffset[]            = { -23505, -41356, -39319, -44537, -39831, -49087, 0};
const int     PoOffset[]            = { 1.312523, 2.124893, 4.947079, 2.392273, 3.731955, 3.401161, -6.178052};
const double  enc2rad               = ((2.0*PI)/0x00010000); // �P��]��16384�p���X�~4���{


int DeviceInit()
{
  if( GlobalValueInit() == ERROR ) {
    printf("[DEVICE INFO] Value Init: ERROR\n");
    return ERROR;
  }

  if( BoardInit() == ERROR )  {
    printf("[DEVICE INFO] Board Init: ERROR\n");
    return ERROR;
  }

  AllBKon(TRUE);

  if( EncAbsPos() == ERROR ) {
    printf("[DEVICE INFO] Recevie Amount of Rotation: ERROR\n");
    return ERROR;
  }

  delay(100);

  if( SRDYcheck(TRUE) == ERROR )  {
    printf("[DEVICE INFO] S-RDY check: ERROR\n");
    return ERROR;
  }

  // DAboard Init 0V
  LightBulbOn(TRUE);

  return OK;
}


int DeviceStart()
{
  const int   ControlScheme = 1;                //�������
  const BYTE  servo_on      = 0b01111111;

  BOOL state_err  = FALSE;

  if( ESTOPcheck(FALSE) == ERROR ) state_err = TRUE;
  if( SRDYcheck(FALSE)  == ERROR ) state_err = TRUE;
  if( ALMcheck(FALSE)   == ERROR ) state_err = TRUE;
  if( TGONcheck(FALSE)  == ERROR ) state_err = TRUE;
  // if( BKcheck(FALSE)    == ERROR ) state_err = TRUE;

  if(state_err == TRUE) {
    printf("[DEVICE INFO] start error!\n");
    is_start = FALSE;
    return ERROR;
  }
  else {
    CSELout(ControlScheme);
    PoWrite_byte(0x00, servo_on);

    printf("[DEVICE INFO] servo on\n");
    is_start = TRUE;
  }

  return OK;
}


int DeviceUpdate()
{
  const BYTE  servo_off     = 0b00000000;

  BOOL err        = FALSE;
  BOOL state_err  = FALSE;
  BOOL limit_err  = FALSE;

  if( is_start == FALSE ) {
    // printf("[DEVICE INFO] start: FALSE\n");
    err = TRUE;
  }

  PositionInput(cur_pos, pre_pos);                //update cur_pos and pre_pos
  PosToVel(cur_pos, pre_pos,cur_vel, pre_vel);    //update cur_vel and pre_vel

  limit_err = LimitPosVelCheck(cur_pos,cur_vel);
  if( limit_err == ERROR) {
    err = TRUE;
  }

  if( ESTOPcheck(FALSE) == ERROR ) state_err = TRUE;
  if( SRDYcheck(FALSE)  == ERROR ) state_err = TRUE;
  if( ALMcheck(FALSE)   == ERROR ) state_err = TRUE;
  // if( TGONcheck(FALSE)  == ERROR ) state_err = TRUE;
  // if( BKcheck(FALSE)    == ERROR ) state_err = TRUE;

  if(state_err == TRUE) {
    // printf("[DEVICE INFO] status error\n");
    err = TRUE;
  }

  if (!err) {
    AllBKon(is_BK);
    TorqCmdOut(cmd_torq);                 // output torque comand
  }
  else {
    PoWrite_byte(0x00, servo_off);        // servo off
    AllBKon(TRUE);
  }

  if(err) return ERROR;
  else    return OK;
}


int DeviceFin()
{
  int     i;
  double  no_torq[7];

  is_start = FALSE;

  for(i=0;i<7;i++) no_torq[i] = 0;
  TorqCmdOut(no_torq);                            // �S���g���N�w��0V
  PoWrite_byte(0x00, 0x00);                       // �S���T�[�{�I�t /S-ON(���_��) 
  AllBKon(TRUE);

  LightBulbOn(FALSE);
  BoardFin();                                     // �e�{�[�h�̏I������

  return OK;
}


int GlobalValueInit()
{
  int i;

  is_start    = FALSE;
  is_BK       = TRUE;

  for(i=0;i<7;i++) {
    cur_pos[i]  = 0;
    pre_pos[i]  = 0;

    cur_vel[i]  = 0;
    pre_vel[i]  = 0;

    pre_torq[i] = 0;
    cur_torq[i] = 0;
    cmd_torq[i] = 0;
    cmd_volt[i] = 0;
  }

  return OK;
}


int BoardInit(void)
{
  int err = 0;

  // CNT�{�[�h�̏�����
  if(CntInit() == FALSE) {
    err = 1;
    printf("[DEVICE INFO] CntInit ERROR!\n");
  }

  // DA�{�[�h�̏�����
  if(DaInit() == FALSE) {
    err = 1;
    printf("[DEVICE INFO] DaInit ERROR!\n");
  }

  // DIO�{�[�h�̏�����
  if(DioInit() == FALSE) {
    err = 1;
    printf("[DEVICE INFO] DioInit ERROR!\n");
  }

  // PI�{�[�h�̏�����
  if(PiInit() == FALSE) {
    err = 1;
    printf("[DEVICE INFO] PiInit ERROR!\n");
  }

  // PO�{�[�h�̏�����
  if(PoInit() == FALSE) {
    err = 1;
    printf("[DEVICE INFO] PoInit ERROR!\n");
  }

  // // COM1(�V���A���|�[�g)�̏�����
  // if(Com1Init() == FALSE) {
  //   err = 1;
  //   printf("[DEVICE INFO] Com1Init ERROR!\n");
  // }

  if(err) return ERROR;
  else    return OK;
}


int BoardFin()
{
  //�v�C��
  int err = 0;

  PoFin();
  PiFin();
  DioFin();
  DaFin();
  CntFin();

  if(err) return ERROR;
  else    return OK;
}


int EncAbsPos(void)
{
  int           i,j;
  BYTE          Sen;
  int           err = 0;
  char          rxdata[10];             // unsigned char ���� char�ɕύX 20181020 yoshinaga
  signed short  AbsPos[7];
  const int     max_rx_bytes = 8;
  // const int     max_tx_bytes = 8;

  for (i=0; i<7; i++) {
    AbsPos[i] = 0;                      // �ϐ��̏�����
  }

  // RS-232C�ő���]�ʒu���擾�i��M�j����

  for (i=0; i<7; i++) AbsPos[i] = 0;    // �ϐ��̏�����

  DioWrite_byte(0x06, 0x00);            // SEL���I����Ԃɂ���
  DioWrite_byte(0x04, 0xFF);            // SEN��S��Low�ɂ��� ���_��

  delay(100);                           // 100ms�҂�
  // Com1FifoClr();

  Sen = 0x01;
  for (i=0; i<7; i++) {                 // 7��
    for(j=0;j<max_rx_bytes+1;j++) {     // �ϐ��̏�����
      *(rxdata+j) = 0x00;
    }


    Sen |= (0x01<<i);                   // SEN�͂P�䂸��High�ɂ���
    DioWrite_byte(0x04, ~Sen);          // SEN���o�͂��� ���_��
    delay(100);                         // 100ms�҂�

    // Com1FifoClr();
    DioWrite_byte(0x06, ~i);            // SEL��0x0�`0x6 ���_��

    // err |= Com1Rx(rxdata, max_rx_bytes);
    DioWrite_byte(0x06, 0x00);          // SEL���I����Ԃɂ���

    // // ��M�f�[�^���`�F�b�N����
    // if(((*(rxdata+0)!=0x50) || (*(rxdata+7)!=0x0D))
    //     || ((*(rxdata+1)!=0x2B) && (*(rxdata+1)!=0x2D))) {	// '0x50'�C'0x0D'�ȂǃA�X�L�[�R�[�h
    //   err = 1;
    //   printf("RxData error!\n");
    // }
    // for(j=2; j<7; j++) {
    //   if( (*(rxdata+j) < 0x30) || (*(rxdata+j) > 0x39) ) {	// '0x30'�C'0x39'�ȂǃA�X�L�[�R�[�h
    //     err = 1;
    //     printf("RxData error!\n");
    //   }
    // }

    // AbsPos[i] = atoi(rxdata+1);      // AbsPos0[] : ����]��

    // // ����]�ʒu���`�F�b�N����
    // if( (AbsPos[i] < -38) || (AbsPos[i] > 38) ) {
    //   err = 1;
    //   printf("AbsPos error!\n");
    // }

    delay(50);                          // 50ms�҂�

    // Com1FifoClr();
  }

//   CntRead_c3kai(0, TmpCnt);
//   for (i=0; i<7; i++) {
//     //		printf("Po0[%d] = %d\n", i, TmpCnt[i] - 0x80000000);
//     TmpCnt[i] -= (AbsPos0[i] * 0x00010000);	// �P��]��16384�p���X�~4���{
//   }
// 
//   CntWrite_c3kai(0, TmpCnt);
// 

  delay(100);
  PositionInput(cur_pos, pre_pos);                //update cur_pos and pre_pos


  if(err) return ERROR;
  else    return OK;
}


int ESTOPcheck(int isInit)
{
  // ����~�{�^���̏�Ԃ�ǂ�
  // �ʏ퓮�쎞��SW�N���[�Y�iHigh�j�A��������SW�I�[�v���iLow�j
  /***********************************************************/
  /* PI�{�[�h�͔���~�{�^���̏�Ԃ��擾����                */
  /* �܂��CPI�{�[�h�̓A���[���R�[�h���擾����                */
  /* �A���[���R�[�h��3�r�b�g�@�ł��A���[���R�[�h�͂�������   */
  /* PI�{�[�h����̃f�[�^�����ł͔��f�ł��Ȃ��E�E�D          */
  /* ���Ȃ��Ƃ��ǂꂩ�̃r�b�g��Low���ƈُ�Ȃ��Ƃ͊m�� kamei */
  /***********************************************************/
  //20181020 PI�{�[�h�ɂ��ALM�M���̓ǂݎ��͖����� yoshinaga

  int   err = 0;
  BYTE  estop;

  static BYTE   pre_estop  = 0x01;

  estop = PiRead_byte(0x07);


  if((estop != 0x01) && (pre_estop == 0x01)) {  // 0xFF : ���x���߃G���[
    err = 1;
    printf("[DEVICE INFO] Emergency Switch is pressed down!\n");
  }
  else if((estop == 0x01) && (pre_estop != 0x01)) {
    printf("[DEVICE INFO] Emergency Switch is released!\n");
  }


  if(isInit == TRUE)  pre_estop = 0x00;
  else                pre_estop = estop;

  if(err) return ERROR;
  else    return OK;
}


int SRDYcheck(int isInit)
{
  // /S-RDY��ǂݏo��
  // S-RDY : �T�[�{�p�b�N���T�[�{�I��(/S-ON)�M������t�\���ǂ����ǂ�
  // 0xC0�F�T�[�{���f�B(6��)�C0x80�F�T�[�{���f�B(7��)

  int   err = 0;
  BYTE  srdy;

  static BYTE   pre_srdy  = 0x80;        // /S-RDY����


  // S-RDY(�T�[�{���f�B�o�͐M��)�̃`�F�b�N
  srdy = ~DioRead_byte(0x00);

  //If each servo is ready, srdy become
  //[6axis] 0xC0 (1100 0000)
  //[7axis] 0x80 (1000 0000)
  if(srdy == 0xFF) {
    err = 1;
    if(pre_srdy == 0x80) 
      printf("[DEVICE INFO] S-RDY error: Check controller power.\n");
  }
  else if(srdy != 0x80) {
    err = 1;
    if(pre_srdy == 0x80)
      printf("[DEVICE INFO] S-RDY %02x error!\n", srdy );
  }


  if(isInit == TRUE)  pre_srdy = 0x80;
  else                pre_srdy = srdy;

  if(err) return ERROR;
  else    return OK;
}


int ALMcheck(int isInit)
{
  // ALM��ǂݏo��
  // ALM : �ُ�����o�����Ƃ��ɁC�I�t(�J)�ɂ���
  // 0xFE�F�ُ�Ȃ� 0xFF�F�A���[������
  // If system have something error, alm become
  // 0xFE (1111 1110)

  int   err = 0;
  BYTE  alm;

  static BYTE   pre_alm = 0xFE;        // /ALM����


  //alm�̃`�F�b�N
  alm = ~DioRead_byte(0x01);

  if(alm != 0xFE) {
    err = 1;
    if(pre_alm == 0xFE)
      printf("[DEVICE INFO] ALM error!\n");
  }


  if(isInit == TRUE)  pre_alm = 0xFE;
  else                pre_alm = alm;

  if(err) return ERROR;
  else    return OK;
}


int TGONcheck(int isInit)
{
  int   err = 0;
  BYTE  tgon;

  static BYTE   pre_tgon = 0xFF;        // /S-RDY����


  // /T-GON�̃`�F�b�N
  tgon = ~DioRead_byte(0x02);

  // /TGON��ǂݏo��
  // TGON : �T�[�{���[�^���ݒ�l�ȏ�̉�]���x�ɂȂ����ꍇ�I��(��)����
  // 0xFF�FJ1�`J6��~���A0xC0�FJ1�`J6��]��
  // ���x���߃G���[�Ɖߋ��̃v���O�����ɂ̓R�����g���Ă��邪�C�G���[�łȂ��Ƃ���TGON��High�ɂȂ�? 20181021 yoshinaga
  if((tgon != 0xFF) && (pre_tgon == 0xFF)) {  // 0xFF : ���x���߃G���[
    printf("[DEVICE INFO] Motor accelerate (TGON)\n");
  }
  else if((tgon == 0xFF) && (pre_tgon != 0xFF)) {
    printf("[DEVICE INFO] Motor decelerate (TGON)");
  }


  if(isInit == TRUE)  pre_tgon = 0xFF;
  else                pre_tgon = tgon;

  if(err) return ERROR;
  else    return OK;
}


int BKcheck(int isInit)
{
  //�����ꂩ�̃T�[�{���u���[�L���I���̂Ƃ��COK��Ԃ�

  int   err = 0;
  BYTE  bk;

  static BYTE   pre_bk = 0x80;        // /S-RDY����


  // /BK��ǂݏo��
  // 0x80: J1-7�u���[�L�I�t, 0xFF: �u���[�L�I��
  bk = ~DioRead_byte(0x03);

  if(bk == 0xFF) {
    err = 1;
    if(pre_bk != 0xFF)
      printf("[DEVICE INFO] Bk = %2x\n", bk);
  }


  if(isInit == TRUE)  pre_bk = 0x80;
  else                pre_bk = bk;

  if(err) return ERROR;
  else    return OK;
}


int CSELout(int ControlScheme)
{
  // C-SEL�̏o��
  // ���������ݒ肷��
  if(ControlScheme == 0) {
    PoWrite_byte(0x01, 0x00);      // �g���N����i�g�����W�X�^���I�t����High���o�́j
    // printf("Torque Control!! \n");
  }
  else if(ControlScheme == 1) {
    PoWrite_byte(0x01, 0x7F);      // ���x����i�g�����W�X�^���I������Low���o�́j
    // printf("Speed Control!! \n");
  }
  else {
    printf("[DEVICE INFO] Control Scheme ERROR!\n");
  }

  return OK;
}


int PositionInput(double CurActAgl[], double PreActAgl[])
{
  int i;

  DWORD CurCnt[8];   // �J�E���^�{�[�h����l��ǂݏo�����߂̃����� 

  // ���݂̃p���X�ʒu�̎擾
  CntRead_c3kai(CurCnt);

  // 0x80000000���ʗ�������
  for ( i=0; i<7; i++ ) {
    PreActAgl[i] = CurActAgl[i];

    CurCnt[i] -= 0x80000000;
    // ���݂̃J�E���g�l����֐ߊp�x���Z�o
    CurActAgl[i] = (long)CurCnt[i]*enc2rad*RRatio[i] - PoOffset[i]*DEG2RAD;
    //CurActAgl[i] = (long)CurCnt[i]*enc2rad*RRatio[i];
  }

  return OK;
}


int PosToVel(double cpos[], double ppos[],double cvel[], double pvel[])
{
  int i;

  for(i=0;i<7;i++) {
    pvel[i] = cvel[i];
    cvel[i] = (cpos[i] - ppos[i]) * FREQ;
  }

  return OK;
}


int LimitPosVelCheck(double pos[],double vel[])
{
  int   i;
  int  limit_err      = OK;
  int  hard_limit_err = OK;

  limit_err |= LimitPosCheck(pos);
  limit_err |= LimitVelCheck(vel);

  if(limit_err == ERROR) {
    hard_limit_err |= HardLimitPosCheck(pos);
    hard_limit_err |= HardLimitVelCheck(vel);

    if(hard_limit_err == ERROR) {
      printf("\n");
      printf("[DEVICE INFO] !!! FATAL ERROR: HARD LIMIT OVER !!!\n");
      printf("[DEVICE INFO] !!! YOU HAVE TO REPORT THIS ERROR TO TRAINERS !!!\n");
      printf("\n");
    }
    else {
      printf("\n");
      printf("[DEVICE INFO] ! ERROR : LIMIT OVER !\n");
      printf("\n");
    }

    printf("[DEVICE INFO] CURRENT POSITION[DEG]:");
    for(i=0;i<7;i++) printf(" %f,",pos[i]*RAD2DEG);
    printf("\n");

    printf("[DEVICE INFO] CURRENT VELOCITY[DEG/SEC]:");
    for(i=0;i<7;i++) printf(" %f,",vel[i]*RAD2DEG);
    printf("\n");

    return ERROR;
  }

  return OK;
}


int LimitPosCheck(double pos[])
{
  int i;

  for (i=0;i<7;i++) {
    if ( pos[i] > max_pos_limit[i]  || pos[i] < min_pos_limit[i]) {
      printf("[DEVICE INFO] position limit: pos[%d] = %f\n",i,pos[i]*RAD2DEG);
      return ERROR;
    }
  }

  return OK;
}


int LimitVelCheck(double vel[])
{
  int i;

  for (i=0;i<7;i++) {
    if ( fabs(vel[i]) > vel_limit[i]) {
      printf("[DEVICE INFO] velocity limit: vel[%d] = %f[deg/s]\n",i,vel[i]*RAD2DEG);
      return ERROR;
    }
  }

  return OK;
}


int HardLimitPosCheck(double pos[])
{
  int i;

  for (i=0;i<7;i++) {
    if ( pos[i] > hard_max_pos_limit[i]  || pos[i] < hard_min_pos_limit[i]) {
      printf("[DEVICE INFO] !!! POSITION HARD LIMIT !!!\n");
      printf("[DEVICE INFO] !!! joint number: %d    !!!\n",i);
      printf("[DEVICE INFO] !!! current pos : %f[deg] !!!\n",pos[i]);
      if ( pos[i] > hard_max_pos_limit[i])  printf("[DEVICE INFO] !!! max pos     : %f[deg] !!!\n",hard_max_pos_limit[i]);
      else                                  printf("[DEVICE INFO] !!! min pos     : %f[deg] !!!\n",hard_min_pos_limit[i]);

      return ERROR;
    }
  }

  return OK;
}


int HardLimitVelCheck(double vel[])
{
  int i;

  for (i=0;i<7;i++) {
    if ( fabs(vel[i]) > hard_vel_limit[i]) {
      printf("[DEVICE INFO] !!! VELOCITY HARD LIMIT !!!\n");
      printf("[DEVICE INFO] !!! joint number: %d    !!!\n",i);
      printf("[DEVICE INFO] !!! current vel : %f[deg/s] !!!\n",vel[i]*RAD2DEG);
      if ( vel[i] > hard_vel_limit[i])  printf("[DEVICE INFO] !!! max vel     : %f[deg/s] !!!\n",hard_vel_limit[i]*RAD2DEG);
      else                              printf("[DEVICE INFO] !!! min vel     : %f[deg/s] !!!\n",-hard_vel_limit[i]);

      return ERROR;
    }
  }

  return OK;
}


int TorqCmdOut(double arg_torq[])
{
  int     i;
  double  torq_out[7];
  double  volt[7];
  const double max_torq_volt = 3;

  for ( i=0; i<7; i++ ) {
    torq_out[i] = -arg_torq[i] * RRatio[i];
    // torq_out[i] = arg_torq[i];

    if(torq_out[i] > torq_limit[i])       torq_out[i] =  torq_limit[i];
    else if(torq_out[i] < -torq_limit[i]) torq_out[i] = -torq_limit[i];

    volt[i]     = (torq_out[i] / hard_torq_limit[i]) * max_torq_volt;
    cmd_volt[i] = volt[i];
  }

  //3V�^����ƃ��[�^�͒�i�g���N���o�͂���
  //���̓d����^����Ƌt��]
  DaOut_c3kai(volt);

  return OK;
}


int LightBulbOn(int on)
{
  if(on)  PoWrite_byte(0x02, 0x01);	// �d���I��
  else    PoWrite_byte(0x02, 0x00);	// �d���I�t

  return OK;
}


void D_GetPosition(double *pos)
{
  // �e�֐߂̌��݊p�x�����擾����
  // �߂�l: rad
  double  dumy[7];

  PositionInput(pos,dumy);
}


// //�u���[�LOFF���w�߂��C�����̎w�߃g���N�ɂ���
void D_SetTorq(double *torq)
{
  //�w�߃g���N[Nm]���w�ߗp�o�b�t�@�ɃZ�b�g����
  int  i;

  for(i=0;i<7;i++) cmd_torq[i] = torq[i];
  is_BK = FALSE;

  return;
}


void GetCmdVolt(double *volt)
{
  // �e�T�[�{�p�b�N�ւ̃g���N�w�ߒl(�d��)��Ԃ�
  // �߂�l: [V]
  int i;

  for(i=0;i<7;i++)
    volt[i] = cmd_volt[i];
}


void GetPulse(double *cur_pulse)
{
  // �e�T�[�{�p�b�N�ւ̃g���N�w�ߒl(�d��)��Ԃ�
  // �߂�l: [V]
  int i;

  for(i=0;i<7;i++)
    cur_pulse[i] = (long)cur_pos[i]/enc2rad/RRatio[i];
}


// //�u���[�LON���w�߂��C�w�߃g���N��0�ɂ���
void D_NoTorq(void)
{
  int     i;

  for(i=0;i<7;i++) cmd_torq[i] = 0;
  is_BK = TRUE;

  return;
}


void AllBKon(int is_bk)
{
  if(is_bk) DioWrite_byte(0x00, 0x00);
  else      DioWrite_byte(0x00, 0x01);
}

