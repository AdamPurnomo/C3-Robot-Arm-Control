/*****************************************************

    C3 Control Program ( 1-Axis Only )

******************************************************/


#include "include.h"
#include "main_cfg.h"
#include "cfg_arm.h"
#include "params_arm.h"
#include "./Arcnet/arc_pci.h"
#include "timer.h"

struct params motorp;
int brakeoff_joint;
int ctrlEndFlag = 0;
int ctrltrig = OFF;

const double inertia[7] = {INERTIA1, INERTIA2, INERTIA3, INERTIA4, INERTIA5, INERTIA6, INERTIA7};
const double Kd[7] = {KD1, KD2, KD3, KD4, KD5, KD6, KD7};
const double Kp[7] = {KP1, KP2, KP3, KP4, KP5, KP6, KP7};
const double joint_limit[2] = {175.*DEG2RAD, 85.*DEG2RAD};
const double max_torque[7]    =  {MAX_TORQUE1, MAX_TORQUE2, MAX_TORQUE3, MAX_TORQUE4, MAX_TORQUE5, MAX_TORQUE6, MAX_TORQUE7} ;
struct path    path_j[7];
struct status  cur_j[7], des_j[7];
double torque[7];

void init(void);
int initializeAll(void);
void initializeData(void);
void start(void);
int ctrlTask(struct params *motor);
void fin(void);
void jointCtrl(struct params *param, int trig);
void getCurrentPosition(struct status *Cur_j);
void pathInit_j(double *Start, double *Destination, double Time);
void pathGenerate_j(struct status *Des_j, unsigned long Time);
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
void All_OFFBrake(void);
void OFFBrake(void);
void ONBrake(void);
void choosemode (void);

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
	for(i=0;i<7;i++)
    {
        motorp.desPos[i] = 0.;
        if(i==3) des_j[i].pos = 90.;
        else des_j[i].pos    = 0.;
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
            angle[i] = des_j[i].pos;
        }
        break;
    default:
        break;
}

for(i=0;i<7;i++)
{
    Cur_j[i].pos = angle[i];
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
if(Time%1000==0) printf("%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", dumt+realt, cur_j[0].pos, cur_j[1].pos, cur_j[2].pos, cur_j[3].pos, cur_j[4].pos, cur_j[5].pos, cur_j[6].pos);
}

void
pdCtrl(struct status *Des_j, struct status *Cur_j)
{

double accel[7];
int i=0;
for(i=0; i<7; i++)
{
    accel[i] = Des_j[i].acc + Kd[i] * (Des_j[i].vel - Cur_j[i].vel) + Kp[i] * (Des_j[i].pos - Cur_j[i].pos);
    torque[i] = inertia[i] * accel[i];

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


int
main(void)
{

    cp(NULL, NULL);
    return(OK);

}



