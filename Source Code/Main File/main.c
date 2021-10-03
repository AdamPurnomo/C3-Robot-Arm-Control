/*****************************************************

    C3 Control Program ( 1-Axis Only )

******************************************************/


#include "include.h"
#include "main_cfg.h"
#include "cfg_arm.h"
#include "params_arm.h"
#include "./Arcnet/arc_pci.h"
#include "timer.h"
#include "kinematics-Test.c"
#include "PseudoJacob.c"
#define g 9.75


struct params motorp;
int brakeoff_joint;
int ctrlEndFlag = 0;
int ctrltrig = OFF;

const double inertia[7] = {INERTIA1, INERTIA2, INERTIA3, INERTIA4, INERTIA5, INERTIA6, INERTIA7};
const double Kd[7] = {KD1, KD2, KD3, KD4, KD5, KD6, KD7};
const double Kp[7] = {KP1, KP2, KP3, KP4, KP5, KP6, KP7};
const double joint_limit[2] = {175.*DEG2RAD, 85.*DEG2RAD};
const double max_torque[7]    =  {MAX_TORQUE1, MAX_TORQUE2, MAX_TORQUE3, MAX_TORQUE4, MAX_TORQUE5, MAX_TORQUE6, MAX_TORQUE7} ;
const double mass[7] = {5.87, 4.65, 4.71, 1.97, 1.62, 1.01, 0.073};
const double length[7] = {0.1, 0.15, 0.25, 0.063, 0.287, 0.06, 0.005};
const double com[7] = {0.048, 0.075, 0.14, 0.02, 0.175, 0.02, 0.0025};
struct path    path_j[7];
struct linepath lpath;
struct circlepath cpath;
struct status  cur_j[7], des_j[7];
double torque[7];

void init(void);
int initializeAll(void);
void initializeData(void);
void start(void);
int ctrlTask(struct params *motor);
void fin(void);
void jointCtrl(struct params *param, int trig);
void lineCtrl (struct params *param, int trig);
void circleCtrl(struct params *param, int trig);
void RMRlineCtrl (struct params *param, int trig);
void RMRcircleCtrl (struct params *param, int trig);
void getCurrentPosition(struct status *Cur_j);
void pathInit_j(double *Start, double *Destination, double Time);
void lineInit (double *Start, double Time);
void circleInit(double *Start, double Time);
void RMRlineInit (double *Start, double Time);
void RMRcircleInit (double *Start, double Time);
void pathGenerate_j(struct status *Des_j, unsigned long Time);
void linepathGenerate (struct status *Des_j, unsigned long Time);
void circlepathGenerate (struct status *Des_j, unsigned long Time);
void RMRlinepathGenerate (struct status *Des_j, unsigned long Time);
void RMRcirclepathGenerate (struct status *Des_j, unsigned long Time);
void pdCtrl(struct status *Des_j, struct status *Cur_j);
void allzero(void);
void allboff(void);
void allbon(void);
void boff1(void);
void boff2(void);
void boff3(void);
void boff4(void);
void boff5(void);
void boff6(void);
void boff7(void);
void control(struct params *motor);
void allbrakeoff(void);
void brakeoff(int joint);
void Nop(void);
int endTask(void);
void joint_moveto(double *Angle,double Time);
void line_moveto (double *Descor, double Time);
void RMRline_moveto(double *Descor, double Time);
void circle_moveto (double *Center, double Time);
void RMRcircle_moveto (double *Center, double Time);
void All_OFFBrake(void);
void OFFBrake(void);
void ONBrake(void);
void choosemode (void);
void FK (double th[7], double T[4][4]);
void IK (double [7], double T[4][4], double pt[7]);
void ResolvedMotionRate(double jointvel[7], double carvel[3], double theta[7]);
void Printmatrix(double A[4][4]);
void jointmode(void);
void linemode(void);
void circlemode(void);
void RMRlinemode(void);
void RMRcirclemode(void);
void grav_comp(double tg[7], double theta[7]);
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
extern thread_pool_t * cp (int argc, char **argv);

/*----------------------------------------------------------------------*/

void
init(void)
{
	ThreadCtl(_NTO_TCTL_IO,0);

	if ((chid = ChannelCreate (0)) == -1){
		fprintf (stderr,"timer.c: couldn't create channel!\n");
		perror (NULL);
		exit (EXIT_FAILURE);
	}

	setupTimer();

	initializeAll();

}


int
initializeAll(void)
{
	char key[16];


	printf("\n\t***** Choose the Control Mode *****\n\n");
	printf("\tOperation or Simulation ? [o/s] \n\n");
	fgets(key, sizeof(key), stdin);

	if (*key == 'o') {
		CtrlMode = operation;
		arcInit();
		printf("\n\tControl Mode -> Operation \n\n");
	}
	else if (*key == 's') {
		CtrlMode = simulation;
		printf("\n\tControl Mode -> Simulation \n\n");
	}

	else {
		CtrlMode = simulation;
		printf("\n\tControl Mode -> Simulation \n\n");
	}


	initializeData();

	return OK;
}

void
initializeData(void)
{

	motorp.mode = nop;
	int i=0;
	for (i = 0; i < 7; i++)
	{
		motorp.desPos[i] = 0.;
		if (i == 3) des_j[i].pos = -90.*DEG2RAD;
        else des_j[i].pos = 0.;
	}
	motorp.desTime = 0.;
	memset(&path_j, 0, sizeof(path_j));

}

void
start(void)
{
	arcnet_start();
    timer_settime(timerid, 0, &timer, NULL);
    ctrltrig = 1;
    ctrlEndFlag = 1;
    ctrlTask(&motorp);
}

int
ctrlTask(struct params *motor)
{
	int rcvid=1;
	MessageT msg;
	static unsigned long ticks = 0;
	pthread_attr_t attr;

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED );

	while(ctrltrig){
		rcvid = MsgReceive(chid, &msg, sizeof(msg), NULL);
		if(rcvid == 0){


			if(!ctrlEndFlag){
				printf("ERROR : Control Task is out of time.\n        fin\n");
				fin();
				return (ERROR);
			}
			ctrlEndFlag = 1;

			pthread_create(NULL,&attr,(void *)control,motor);
			ticks++;

		}
	}

	return (EXIT_SUCCESS);

}

void
control(struct params *motor)
{

	int stat1;
	static struct params param;
	static int mode = nop;
	static int joint;
	static int trig;


	if( pthread_mutex_lock ( &mutex ) != EOK ){
		printf(" ERROR : mutex cannnot lock.\n        fin\n");
		fin();
		return;
    }

	switch(CtrlMode){
	case operation:
		stat1 = iSend_C();
		break;
	case simulation:
		break;

	default:
		break;
	}

	memcpy(&param, motor, sizeof(struct params));

	getCurrentPosition(cur_j);


	if(param.trig){
		trig = TRUE;
		motor->trig = FALSE;
	}
	else
		trig = FALSE;

	mode = param.mode;
	joint = param.joint;

	switch(mode){

	case joint_mode:
		jointCtrl(&param, trig);
		break;
	case allbrakeoff_mode:
		allbrakeoff();
		break;
	case brakeoff_mode:
		brakeoff(joint);
		break;
    case line_mode:
        lineCtrl(&param, trig);
        break;
    case circle_mode:
        circleCtrl(&param, trig);
        break;
    case RMRline:
        RMRlineCtrl(&param, trig);
        break;
    case RMRcircle:
        RMRcircleCtrl(&param, trig);
        break;
	default:
		Nop();
		break;
	}

	switch(CtrlMode){
	case operation:
    stat1 = RecData();
    if(stat1 == ERROR) {
      fin();
      return;
    }
		break;
	case simulation:
		break;

	default:
		break;
	}

	endTask();
	return;

}

void
jointCtrl(struct params *param, int trig)
{

	static unsigned long tick;
	static double initPos[7], desPos[7],desTime;


	if(trig){

        int i=0;
		for(i=0;i<7;i++)
		{
		    desPos[i] = param->desPos[i];
            initPos[i] = cur_j[i].pos;
		}
		 desTime = param->desTime;
		 pathInit_j(initPos, desPos, desTime);
		 tick = 0;
    }


	pathGenerate_j(des_j, tick);
	tick++;
	pdCtrl(des_j, cur_j);

    return;
}

void
lineCtrl(struct params *param, int trig)
{
    static unsigned long tick;
	static double initPos[7],desTime;
	int i;


	if(trig){
		for(i=0;i<7;i++)
		{
            initPos[i] = cur_j[i].pos;
		}
		 desTime = param->desTime;
		 lineInit(initPos,desTime);
		 tick = 0;
    }


	linepathGenerate(des_j,tick);
	pdCtrl(des_j, cur_j);
	getCurrentPosition(cur_j);

    tick++;

    return;
}

void RMRlineCtrl(struct params *param, int trig)
{
    static unsigned long tick;
	static double initPos[7],desTime;
	int i;

	if(trig){
		for(i=0;i<7;i++)
		{
            initPos[i] = cur_j[i].pos;
		}
		 desTime = param->desTime;
		 RMRlineInit(initPos,desTime);
		 tick = 0;
    }

    RMRlinepathGenerate(des_j,tick);
	pdCtrl(des_j, cur_j);
	getCurrentPosition(cur_j);

    tick++;

    return;

}

void
circleCtrl(struct params *param, int trig)
{
    static unsigned long tick;
	static double initPos[7],desTime;


	if(trig){

        int i=0;
		for(i=0;i<7;i++)
		{
            initPos[i] = cur_j[i].pos;
		}
		 desTime = param->desTime;
		 circleInit(initPos,desTime);
		 tick = 0;
    }


	circlepathGenerate(des_j,tick);
	pdCtrl(des_j, cur_j);
    tick++;

    return;
}

void RMRcircleCtrl(struct params *param, int trig)
{
    static unsigned long tick;
	static double initPos[7],desTime;


	if(trig){

        int i=0;
		for(i=0;i<7;i++)
		{
            initPos[i] = cur_j[i].pos;
		}
		 desTime = param->desTime;
		 RMRcircleInit(initPos,desTime);
		 tick = 0;
    }


	RMRcirclepathGenerate(des_j,tick);
	pdCtrl(des_j, cur_j);
    tick++;

    return;
}

void
getCurrentPosition(struct status *Cur_j)
{

double angle[7] = {0., 0., 0., 0., 0., 0., 0} ;
static double pre_j_pos[7] = {0., 0., 0., 0., 0., 0., 0};
static double pre_j_vel[7] = {0., 0., 0., 0., 0., 0., 0};
int i=0;
/*--- get joint angles ---*/
/*--- Selection of control mode ---*/

switch(CtrlMode){
    case operation:
        GetPosition(angle);
        break;
    case simulation:
        for(i=0;i<7;i++)
        {
            if(i==3) angle[i] = des_j[i].pos + (DEG2RAD*90);
            else angle[i]=des_j[i].pos;
        }
        break;
    default:
        break;
}

for(i=0;i<7;i++)
{
    if(i==3)
    {
        Cur_j[i].pos = angle[i]-(90*DEG2RAD);
    }
    else
    {
        Cur_j[i].pos = angle[i];
    }
    Cur_j[i].vel = (Cur_j[i].pos - pre_j_pos[i]) / TICKS;
    Cur_j[i].acc = (Cur_j[i].vel - pre_j_vel[i]) / TICKS;
    pre_j_pos[i] = Cur_j[i].pos;
    pre_j_vel[i] = Cur_j[i].vel;
}

}

void
pathInit_j(double *Start, double *Destination, double Time)
{
double t3, t4, t5;
double diff[7];
int i=0;

t3 = Time * Time * Time;
t4 = t3 * Time;
t5 = t4 * Time;

for(i=0; i<7;i++)
{
    diff[i] = Destination[i] - Start[i];
    path_j[i].pos[0] = Start[i];
    path_j[i].pos[3] =  10. * diff[i] / t3;
    path_j[i].pos[4] = -15. * diff[i] / t4;
    path_j[i].pos[5] =   6. * diff[i] / t5;

    path_j[i].vel[2] = 3. * path_j[i].pos[3];
    path_j[i].vel[3] = 4. * path_j[i].pos[4];
    path_j[i].vel[4] = 5. * path_j[i].pos[5];

    path_j[i].acc[1] =  6. * path_j[i].pos[3];
    path_j[i].acc[2] = 12. * path_j[i].pos[4];
    path_j[i].acc[3] = 20. * path_j[i].pos[5];
    path_j[i].time = Time;
}

}

void
lineInit(double *Start, double Time)
{
    double t3, t4, t5;
    double diff[3];
    int i=0;
    t3 = Time * Time * Time;
    t4 = t3 * Time;
    t5 = t4 * Time;

    FK(Start,lpath.T);
    for(i=0;i<7;i++)
    {
        lpath.pre_pos[i]=Start[i];
        lpath.pre_vel[i]=0;
    }

    for(i=0;i<3;i++)
    {
        lpath.a[i] =lpath.T[i][3];
        diff[i]=lpath.d[i]-lpath.a[i];

        lpath.pos[i][0] = lpath.a[i];
        lpath.pos[i][3] =  10. * diff[i] / t3;
        lpath.pos[i][4] = -15. * diff[i] / t4;
        lpath.pos[i][5] =   6. * diff[i] / t5;

        lpath.vel[i][2] = 3. * lpath.pos[i][3];
        lpath.vel[i][3] = 4. * lpath.pos[i][4];
        lpath.vel[i][4] = 5. * lpath.pos[i][5];

        lpath.acc[i][1] =  6. * lpath.pos[i][3];
        lpath.acc[i][2] = 12. * lpath.pos[i][4];
        lpath.acc[i][3] = 20. * lpath.pos[i][5];
    }
    lpath.time = Time;
}

void RMRlineInit(double *Start, double Time)
{
    double t3, t4, t5;
    double diff[3];
    int i=0;
    t3 = Time * Time * Time;
    t4 = t3 * Time;
    t5 = t4 * Time;

    FK(Start,lpath.T);
    for(i=0;i<7;i++)
    {
        lpath.pre_pos[i]=Start[i];
        lpath.pre_vel[i]=0;
    }

    for(i=0;i<3;i++)
    {
        lpath.a[i] =lpath.T[i][3];
        diff[i]=lpath.d[i]-lpath.a[i];

        lpath.pos[i][0] = lpath.a[i];
        lpath.pos[i][3] =  10. * diff[i] / t3;
        lpath.pos[i][4] = -15. * diff[i] / t4;
        lpath.pos[i][5] =   6. * diff[i] / t5;

        lpath.vel[i][2] = 3. * lpath.pos[i][3];
        lpath.vel[i][3] = 4. * lpath.pos[i][4];
        lpath.vel[i][4] = 5. * lpath.pos[i][5];

        lpath.acc[i][1] =  6. * lpath.pos[i][3];
        lpath.acc[i][2] = 12. * lpath.pos[i][4];
        lpath.acc[i][3] = 20. * lpath.pos[i][5];
    }
    lpath.time = Time;
}

void circleInit(double *Start, double Time)
{
    double t3, t4, t5;
    int i;
    double diffphase;

    t3 = Time * Time * Time;
    t4 = t3 * Time;
    t5 = t4 * Time;
    diffphase = 360*DEG2RAD;
    FK(Start,cpath.T);

    for(i=0;i<7;i++)
    {
        cpath.pre_pos[i]=Start[i];
        cpath.pre_vel[i]=0;
    }
    cpath.pos[0] = 0;
    cpath.pos[3] =  10. * diffphase / t3;
    cpath.pos[4] = -15. * diffphase / t4;
    cpath.pos[5] =   6. * diffphase / t5;

    cpath.vel[2] = 3. * cpath.pos[3];
    cpath.vel[3] = 4. * cpath.pos[4];
    cpath.vel[4] = 5. * cpath.pos[5];

    cpath.acc[1] =  6. * cpath.pos[3];
    cpath.acc[2] = 12. * cpath.pos[4];
    cpath.acc[3] = 20. * cpath.pos[5];

    cpath.time = Time;

}

void RMRcircleInit(double *Start, double Time)
{
    double t3, t4, t5;
    int i;
    double diffphase;

    t3 = Time * Time * Time;
    t4 = t3 * Time;
    t5 = t4 * Time;
    diffphase = 360*DEG2RAD;
    FK(Start,cpath.T);

    for(i=0;i<7;i++)
    {
        cpath.pre_pos[i]=Start[i];
        cpath.pre_vel[i]=0;
    }
    cpath.pos[0] = 0;
    cpath.pos[3] =  10. * diffphase / t3;
    cpath.pos[4] = -15. * diffphase / t4;
    cpath.pos[5] =   6. * diffphase / t5;

    cpath.vel[2] = 3. * cpath.pos[3];
    cpath.vel[3] = 4. * cpath.pos[4];
    cpath.vel[4] = 5. * cpath.pos[5];

    cpath.acc[1] =  6. * cpath.pos[3];
    cpath.acc[2] = 12. * cpath.pos[4];
    cpath.acc[3] = 20. * cpath.pos[5];

    cpath.time = Time;
}

void
pathGenerate_j(struct status *Des_j, unsigned long Time)
{

double t;
double static realt, dumt;
int i=0;

if(Time ==0) dumt = round(realt) ;
realt = Time*TICKS;

for(i=0; i<7; i++)
{
    t = (Time * TICKS);
    if(t>path_j[i].time)
    {
        t = path_j[i].time;
    }
    Des_j[i].pos = path_j[i].pos[0] + t * t * t * (path_j[i].pos[3] + t * (path_j[i].pos[4] + t * path_j[i].pos[5]));
    Des_j[i].vel = t * t * (path_j[i].vel[2] + t * (path_j[i].vel[3] + t * path_j[i].vel[4]));
    Des_j[i].acc = t * (path_j[i].acc[1] + t * (path_j[i].acc[2] + t * path_j[i].acc[3]));
}
if(Time%1000==0)
    {
    printf("%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", dumt+realt, RAD2DEG*cur_j[0].pos, RAD2DEG*cur_j[1].pos, RAD2DEG*cur_j[2].pos, RAD2DEG*cur_j[3].pos, RAD2DEG*cur_j[4].pos, RAD2DEG*cur_j[5].pos, RAD2DEG*cur_j[6].pos);

}
}

void
linepathGenerate(struct status *Des_j, unsigned long Time)
{
    int i;
    double t;
    double th[7];
    static double cur_th[7], T[4][4];
    for(i=0;i<7;i++) cur_th[i] = cur_j[i].pos;

    t = (Time * TICKS);
    if(t>lpath.time) t = lpath.time;

    for(i=0; i<3; i++)
        {
            lpath.T[i][3]= lpath.pos[i][0] + t * t * t * (lpath.pos[i][3] + t * (lpath.pos[i][4] + t * lpath.pos[i][5]));
        }
    IK(th, lpath.T,cur_th);
	FK(cur_th, T);

    for(i=0;i<7;i++)
    {
        Des_j[i].pos = th[i];
        Des_j[i].vel = (Des_j[i].pos - lpath.pre_pos[i])/TICKS;
        Des_j[i].acc = (Des_j[i].vel - lpath.pre_vel[i])/TICKS;

        lpath.pre_pos[i] = Des_j[i].pos;
        lpath.pre_vel[i] = Des_j[i].vel;
    }
	if (Time% 1000 == 0)
	{
		printf("%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", Time* TICKS, RAD2DEG * des_j[0].pos, RAD2DEG * des_j[1].pos, RAD2DEG * des_j[2].pos, RAD2DEG * des_j[3].pos, RAD2DEG * des_j[4].pos, RAD2DEG * des_j[5].pos, RAD2DEG * des_j[6].pos, RAD2DEG * cur_j[0].pos, RAD2DEG * cur_j[1].pos, RAD2DEG * cur_j[2].pos, RAD2DEG * cur_j[3].pos, RAD2DEG * cur_j[4].pos, RAD2DEG * cur_j[5].pos, RAD2DEG * cur_j[6].pos, torque[0], torque[1], torque[2], torque[3], torque[4], torque[5], torque[6], lpath.T[0][3], lpath.T[1][3], lpath.T[2][3], T[0][3], T[1][3], T[2][3]);
	}
}

void RMRlinepathGenerate(struct status *Des_j, unsigned long Time)
{
    int i;
    double t;
    double omega[7];
    double cur_th[7], T[4][4];
    for(i=0;i<7;i++) cur_th[i] = cur_j[i].pos;

    t = (Time * TICKS);
    if(t>lpath.time) t = lpath.time;

    for (i=0;i<3;i++)
    {
        lpath.T[i][3]= lpath.pos[i][0] + t * t * t * (lpath.pos[i][3] + t * (lpath.pos[i][4] + t * lpath.pos[i][5]));
        lpath.carvel[i] = t * t *(lpath.vel[i][2] + t * (lpath.vel[i][3] + t * (lpath.vel[i][4])));
    }

    ResolvedMotionRate(omega,lpath.carvel,cur_th);

    for(i=0;i<7;i++)
    {
        Des_j[i].vel = omega[i]; /*(Des_j[i].pos - lpath.pre_pos[i])/TICKS;*/
        Des_j[i].pos = lpath.pre_pos[i] + Des_j[i].vel*TICKS;
        Des_j[i].acc = (Des_j[i].vel - lpath.pre_vel[i])/TICKS;

        lpath.pre_pos[i] = Des_j[i].pos;
        lpath.pre_vel[i] = Des_j[i].vel;
    }


    FK(cur_th, T);

	if (Time% 1000 == 0)
	{
		printf("%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", Time* TICKS, RAD2DEG * des_j[0].pos, RAD2DEG * des_j[1].pos, RAD2DEG * des_j[2].pos, RAD2DEG * des_j[3].pos, RAD2DEG * des_j[4].pos, RAD2DEG * des_j[5].pos, RAD2DEG * des_j[6].pos, RAD2DEG * cur_j[0].pos, RAD2DEG * cur_j[1].pos, RAD2DEG * cur_j[2].pos, RAD2DEG * cur_j[3].pos, RAD2DEG * cur_j[4].pos, RAD2DEG * cur_j[5].pos, RAD2DEG * cur_j[6].pos, torque[0], torque[1], torque[2], torque[3], torque[4], torque[5], torque[6], lpath.T[0][3], lpath.T[1][3], lpath.T[2][3], T[0][3], T[1][3], T[2][3], lpath.carvel[0], lpath.carvel[1], lpath.carvel[2]);
	}
}

void
circlepathGenerate(struct status *Des_j, unsigned long Time)
{
    int i;
    double t;
    double th[7];
    static double cur_th[7], T[4][4];
    for(i=0;i<7;i++) cur_th[i] = cur_j[i].pos;


    t = (Time * TICKS);
    if(t>cpath.time) t = cpath.time;

    cpath.phase = cpath.pos[0] + t * t * t * (cpath.pos[3] + t * (cpath.pos[4] + t * cpath.pos[5]));

    cpath.T[0][3] = cpath.a[0];
    cpath.T[1][3] = cpath.a[1] + cpath.r*cos(cpath.phase);
    cpath.T[2][3] = cpath.a[2] + cpath.r*sin(cpath.phase);

    IK(th, cpath.T,cur_th);
    FK(cur_th, T);

    for(i=0;i<7;i++)
    {
        Des_j[i].pos = th[i];
        Des_j[i].vel = (Des_j[i].pos - cpath.pre_pos[i])/TICKS;
        Des_j[i].acc = (Des_j[i].vel - cpath.pre_vel[i])/TICKS;

        cpath.pre_pos[i] = Des_j[i].pos;
        cpath.pre_vel[i] = Des_j[i].vel;
    }
     if(Time%100==0)
    {
        printf("%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", Time*TICKS, RAD2DEG*des_j[0].pos, RAD2DEG*des_j[1].pos, RAD2DEG*des_j[2].pos, RAD2DEG*des_j[3].pos, RAD2DEG*des_j[4].pos, RAD2DEG*des_j[5].pos, RAD2DEG*des_j[6].pos, RAD2DEG*cur_j[0].pos, RAD2DEG*cur_j[1].pos, RAD2DEG*cur_j[2].pos, RAD2DEG*cur_j[3].pos, RAD2DEG*cur_j[4].pos, RAD2DEG*cur_j[5].pos, RAD2DEG*cur_j[6].pos, torque[0], torque [1], torque[2], torque[3], torque[4], torque[5], torque[6], cpath.T[0][3], cpath.T[1][3], cpath.T[2][3], T[0][3], T[1][3], T[2][3], RAD2DEG*cpath.phase);
    }
}

void RMRcirclepathGenerate(struct status *Des_j, unsigned long Time)
{
    int i;
    double t;
    double omega[7];
    static double cur_th[7], T[4][4];
    for(i=0;i<7;i++) cur_th[i] = cur_j[i].pos;


    t = (Time * TICKS);
    if(t>cpath.time) t = cpath.time;

    cpath.phase = cpath.pos[0] + t * t * t * (cpath.pos[3] + t * (cpath.pos[4] + t * cpath.pos[5]));
    cpath.phasevel = t * t *(cpath.vel[2] + t * (cpath.vel[3] + t * (cpath.vel[4])));

    cpath.T[0][3] = cpath.a[0];
    cpath.T[1][3] = cpath.a[1] + cpath.r*cos(cpath.phase);
    cpath.T[2][3] = cpath.a[2] + cpath.r*sin(cpath.phase);
    cpath.carvel[0] = 0;
    cpath.carvel[1] = -cpath.r*sin(cpath.phase)*cpath.phasevel;
    cpath.carvel[2] = cpath.r*cos(cpath.phase)*cpath.phasevel;

    ResolvedMotionRate(omega,cpath.carvel,cur_th);
    FK(cur_th, T);

    for(i=0;i<7;i++)
    {
        Des_j[i].vel = omega[i];
        Des_j[i].pos = cpath.pre_pos[i] + Des_j[i].vel*TICKS;
        Des_j[i].acc = (Des_j[i].vel - cpath.pre_vel[i])/TICKS;

        cpath.pre_pos[i] = Des_j[i].pos;
        cpath.pre_vel[i] = Des_j[i].vel;
    }
     if(Time%100==0)
    {
        /*printf("%lf, %lf, %lf, %lf, %lf, %lf, %lf,%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", cur_th[0]*Rad2Deg, cur_th[1]*Rad2Deg, cur_th[2]*RAD2DEG, cur_th[3]*RAD2DEG, cur_th[4]*RAD2DEG, cur_th[5]*RAD2DEG, cur_th[6]:RAD2DEG,omega[0]*Rad2Deg, omega[1]*RAD2DEG, omega[2]*RAD2DEG, omega[3]*RAD2DEG, omega[4]*RAD2DEG, omega[5]*RAD2DEG, omega[6]*RAD2DEG, cpath.carvel[0], cpath.carvel[1], cpath.carvel[2]);*/
        printf("%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", Time*TICKS, RAD2DEG*des_j[0].pos, RAD2DEG*des_j[1].pos, RAD2DEG*des_j[2].pos, RAD2DEG*des_j[3].pos, RAD2DEG*des_j[4].pos, RAD2DEG*des_j[5].pos, RAD2DEG*des_j[6].pos, RAD2DEG*cur_j[0].pos, RAD2DEG*cur_j[1].pos, RAD2DEG*cur_j[2].pos, RAD2DEG*cur_j[3].pos, RAD2DEG*cur_j[4].pos, RAD2DEG*cur_j[5].pos, RAD2DEG*cur_j[6].pos, torque[0], torque [1], torque[2], torque[3], torque[4], torque[5], torque[6], cpath.T[0][3], cpath.T[1][3], cpath.T[2][3], T[0][3], T[1][3], T[2][3], RAD2DEG*cpath.phase, cpath.carvel[0], cpath.carvel[1], cpath.carvel[2]);
    }
}

void
pdCtrl(struct status *Des_j, struct status *Cur_j)
{

double accel[7];
int i=0;

double tg[7], cur_th[7];
for(i=0;i<7;i++) cur_th[i] = Cur_j[i].pos;
grav_comp(tg,cur_th);

for(i=0; i<7; i++)
{
    accel[i] = Des_j[i].acc + Kd[i] * (Des_j[i].vel - Cur_j[i].vel) + 100*Kp[i] * (Des_j[i].pos - Cur_j[i].pos);
    torque[i] = inertia[i] * accel[i]; /*- tg[i];*/

    if(torque[i] > max_torque[i])
	torque[i] = max_torque[i];
    else if(torque[i] < - max_torque[i])
	torque[i] = - max_torque[i];
}


	switch(CtrlMode){
	    case operation:
	        SetTorq(torque);
	        break;
        case simulation:
	          NoTorq();
	          break;
        default:
            NoTorq();
            break;
            }
	}

int
endTask( void )
{
ctrlEndFlag = 1;
pthread_mutex_unlock ( &mutex );

return (EXIT_SUCCESS);

}

void
allbrakeoff(void)
{
	AllBrakeOFF();
}

void
brakeoff(int joint)
{
	BrakeOFF(joint);

}

void
Nop(void)
{
	NoTorq();
}

void
joint_moveto(double *Angle, double Time)
{
	int i=0;
	for(i=0;i<7;i++) motorp.desPos[i] = Angle[i];
	motorp.desTime = Time;
	motorp.trig    = TRUE;
	motorp.mode    = joint_mode;
}

void line_moveto(double *Descor, double Time)
{
    int i=0;
	for(i=0;i<3;i++) lpath.d[i] = Descor[i];
	motorp.desTime = Time;
	motorp.trig    = TRUE;
	motorp.mode    = line_mode;
}

void RMRline_moveto(double *Descor, double Time)
{
    int i=0;
	for(i=0;i<3;i++) lpath.d[i] = Descor[i];
	motorp.desTime = Time;
	motorp.trig    = TRUE;
	motorp.mode    = RMRline;
}

void circle_moveto(double *Center, double Time)
{
    int i=0;
	for(i=0;i<3;i++) cpath.a[i] = Center[i];
    motorp.desTime = Time;
	motorp.trig    = TRUE;
	motorp.mode    = circle_mode;
}

void RMRcircle_moveto(double *Center, double Time)
{
    int i=0;
	for(i=0;i<3;i++) cpath.a[i] = Center[i];
    motorp.desTime = Time;
	motorp.trig    = TRUE;
	motorp.mode    = RMRcircle;
}

void
All_OFFBrake(void)
{

	motorp.mode = allbrakeoff_mode;
}

void OFFBrake(void)
{

	motorp.mode = brakeoff_mode;
	motorp.joint = brakeoff_joint;
}

void
ONBrake(void)
{
	motorp.mode = nop;
}

void
fin(void)
{
	ctrltrig = 0;
    ONBrake();
    arcFin();
	timer_delete(timerid);
}

void choosemode(void)
{
    int i;
    int j;
    int k;

    printf("Please input the modes\n");
    printf("Press 0 for Circle Mode\n");
    printf("Press 1 for Line Mode\n");
    printf("Press 2 for Joint Mode\n");
    fflush(stdout);
    scanf("%d", &i);
    if(i ==0)
    {

    }
    else if(i==1)
    {
        double t;
        double cord[3],theta[7],T[4][4];
        getCurrentPosition(cur_j);
        for(k=0;k<7;k++)
        {
            theta[k]=cur_j[k].pos;
        }
        FK(theta,T);
        printf("Input the duration of movement (s) ");
        fflush(stdout);
        scanf("%lf", &t);

        printf("Which Axis do you want to draw a line?\n");
        printf("Press 1 for x-axis\n");
        printf("Press 2 for y-axis\n");
        printf("Press 3 for z-axis\n");
        fflush(stdout);
        scanf("%d", &j);

        double x;
        printf("Input the length of line you want to draw in axis you have chosen\n");
        fflush(stdout);
        scanf("%lf", &x);
        for(k=0;k<3;k++)
        {
            cord[k]=T[k][3];
            if(k==j-1)
            {
                cord[j-1] = cord[j-1]+x;
            }
        }
        line_moveto(cord, t);
    }
    else if(i ==2)
    {
        double t;
        double th[7];
        printf("Input the duration of movement (s) ");
        fflush(stdout);
        scanf("%lf", &t);
        for(j=0; j<7; j++)
        {
            printf("Input the final goal of Theta %d (Deg) ", j+1);
            fflush(stdout);
            scanf("%lf", &th[j]);
        }
        th[3]=th[3]+90;
        joint_moveto(th, t);
    }
}

void Printmatrix(double A[4][4])
{
    int i,j;
    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            printf("%lf ", A[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

void linemode(void)
{
	double destime, cord[3], theta[7], T[4][4];
	int i;
	getCurrentPosition(cur_j);
	for (i = 0; i < 7; i++) theta[i] = cur_j[i].pos;
	FK(theta, T);

	for (i = 0; i < 3; i++)
	{
		cord[i] = T[i][3];
		if (i == 1) cord[i] = cord[i] + 150;
	}
	destime = 20;
	line_moveto(cord, destime);
}

void RMRlinemode(void)
{
    double destime, cord[3], theta[7], T[4][4];
	int i;
	getCurrentPosition(cur_j);
	for (i = 0; i < 7; i++) theta[i] = cur_j[i].pos;
	FK(theta, T);

	for (i = 0; i < 3; i++)
	{
		cord[i] = T[i][3];
		if (i == 1) cord[i] = cord[i] + 150;
	}
	destime = 20;
	RMRline_moveto(cord, destime);
}

void jointmode1 (void)
{
    double destime = 10;
    double theta[7] = {-10*DEG2RAD, -15*DEG2RAD, 0*DEG2RAD, -60*DEG2RAD, -10*DEG2RAD, -20*DEG2RAD, -10*DEG2RAD};
    joint_moveto(theta, destime);
}

void jointmode2 (void)
{
    double destime = 10;
    double theta[7] = {10*DEG2RAD, 15*DEG2RAD, 0*DEG2RAD, -100*DEG2RAD, 40*DEG2RAD, -10*DEG2RAD, 40*DEG2RAD};
    joint_moveto(theta, destime);
}

void jointmode3 (void)
{
    double destime = 10;
    double theta[7] = {-10*DEG2RAD, -20*DEG2RAD, 0*DEG2RAD, -80*DEG2RAD, -40*DEG2RAD, -50*DEG2RAD, -60*DEG2RAD};
    joint_moveto(theta, destime);
}


void circlemode (void)
{
    double destime, center[3], T[4][4];
	int i;
	cpath.r = 75;
	destime = 20;
	getCurrentPosition(cur_j);
	for (i = 0; i < 7; i++) cpath.init_th[i] = cur_j[i].pos;
	FK(cpath.init_th, T);
/*Circle is drawn in YZ plane in counter clockwise direction, the center of the circle in Y-axis is the current position subtracted by radius*/
	for (i = 0; i < 3; i++)
	{
		center[i] = T[i][3];
		if (i == 1) center[i] = center[i] - cpath.r;
	}
    circle_moveto(center, destime);
}

void RMRcirclemode(void)
{
    double destime, center[3], T[4][4];
	int i;
	cpath.r = 75;
	destime = 20;
	getCurrentPosition(cur_j);
	for (i = 0; i < 7; i++) cpath.init_th[i] = cur_j[i].pos;
	FK(cpath.init_th, T);
/*Circle is drawn in YZ plane in counter clockwise direction, the center of the circle in Y-axis is the current position subtracted by radius*/
	for (i = 0; i < 3; i++)
	{
		center[i] = T[i][3];
		if (i == 1) center[i] = center[i] - cpath.r;
	}
    RMRcircle_moveto(center, destime);
}

void grav_comp(double tg[7], double theta[7])
{
   double m1, m2, m3, m4, m5, m6, m7;
   double l1, l2, l3, l4, l5, l6, l7;
   double x1, x2, x3, x4, x5, x6, x7;
   double x23, x45, x67;
   double t2, t4, t6;
   int i;

   m1 = mass[0]; m2 = mass[1]; m3 = mass[2]; m4 = mass[3]; m5 = mass[4]; m6 = mass[5]; m7 = mass[6];
   l1 = length[0]; l2 = length[1]; l3 = length[2]; l4 = length[3]; l5 = length[4]; l6 = length[5]; l7 = length[6];
   x1 = com[0]; x2 = com[1]; x3 = com[2]; x4 = com[3]; x5 = com[4]; x6 = com[5]; x7 = com[6];

   x67 = (m6*x6 + m7*(l6+x7))/(m6+m7);
   x45 = (m4*x4 + m5*(l4+x5))/(m4+m5);
   x23 = (m2*x2 + m3*(l2*x3))/(m2+m3);

   t6 = (m6+m7)*x67*sin(theta[1]+theta[3]+theta[5])*cos(theta[4])*g;
   t4 = (m4+m5)*x45*sin(theta[1]+theta[3])*g + (m6+m7)*( (l4+l5)*sin(theta[1]+theta[3]) + x67*sin(theta[1]+theta[3]+theta[5]) )*g;
   t2 = (m2+m3)*x23*sin(theta[1])*g + (m4+m5)*( (l2+l3)*sin(theta[1]) + x45*sin(theta[1]+theta[3]) )*g + (m6+m7)*( (l2+l3)*sin(theta[1]) + (l4+l5)*sin(theta[1]+theta[3]) + x67*(theta[1]+theta[3]+theta[5]))*g;

   for(i=0;i<7;i++)
   {
       tg[i] = 0;
   }
   tg[1] = t2;
   tg[3] = t4;
   tg[5] = t6;
}
int
main(void)
{

    cp(NULL, NULL);
    return(OK);

}



