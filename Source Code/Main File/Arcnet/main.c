/*****************************************************

    PA10 Control Program ( 1-Axis Only )

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

const double inertia[7] = 
	 {INERTIA1, INERTIA2, INERTIA3, INERTIA4, INERTIA5, INERTIA6, INERTIA7};
const double Kd[7] = {KD1, KD2, KD3, KD4, KD5, KD6, KD7};
const double Kp[7] = {KP1, KP2, KP3, KP4, KP5, KP6, KP7};
const double joint_limit[2] = {175.*DEG2RAD, 85.*DEG2RAD};
const double max_torque[7] =
	 {MAX_TORQUE1, MAX_TORQUE2, MAX_TORQUE3, MAX_TORQUE4, MAX_TORQUE5, MAX_TORQUE6, MAX_TORQUE7};
struct path         path_j[7];
struct linepath     lpath;
struct circlepath   cpath;
struct status  cur_j[7], des_j[7];    
double torque[7];

FILE *fp;
char filename[] = "./output.csv";

void init(void);
int initializeAll(void);
void initializeData(void);
void start(void);
int ctrlTask(struct params *motor);
void fin(void);
void jointCtrl(struct params *param, int trig);
void lineCtrl(struct params *param, int trig);
void circleCtrl(struct params *param, int trig);
void getCurrentPosition(struct status *Cur_j);
void pathInit_j(double *Start, double *Destination, double Time);
void pathGenerate_j(struct status *Des_j, unsigned long Time);
void linePathInit(double *Start, double Time);
void linePathGenerate(struct status *Des_j, struct status *Cur_j, unsigned long Time);
void circlePathInit(double *Start, double Time);
void circlePathGenerate(struct status *Des_j, struct status *Cur_j, unsigned long Time);
void pdCtrl(struct status *Des_j, struct status *Cur_j);
void jointmove(void);
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
void line_moveto(double Time);
void circle_moveto(double Time);
void All_OFFBrake(void);
void OFFBrake(void);
void ONBrake(void);
void RecodeData(int endFlag);
int  kine(double th[7],double T[4][4]);
int  invkine(double th[7], double T[4][4]);
double gcompen(double th[7],int i);

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


int initializeAll(void)
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

	//open file
	if ((fp = fopen(filename, "w")) == NULL) {
		fprintf(stderr, "faile open file \n");
		return EXIT_FAILURE;
	}

	initializeData(); 

	return OK;
}

void initializeData(void)
{
	int i;

	for(i=0;i<7;i++) {
		des_j[i].pos = 0;
	}

	for(i=0;i<7;i++) {
		motorp.desPos[i] = 0.;
	}
	motorp.trig    	 = FALSE;
	motorp.mode = nop;
	motorp.desTime = 0.;

	memset(path_j, 0, sizeof(path_j));
	//memset(lpath, 0, sizeof(lpath));
}

void start(void)
{
  arcnet_start();
  
    timer_settime(timerid, 0, &timer, NULL);
    ctrltrig = 1;
    ctrlEndFlag = 1;

    ctrlTask(&motorp);

}

int ctrlTask(struct params *motor)
{
	int rcvid=1;
	MessageT msg;
	static unsigned long ticks = 0;
    pthread_t thread1;
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
			ctrlEndFlag = 0;
			pthread_create(&thread1,&attr,(void *)control,motor);
			ticks++;

		}
	}
  
	return (EXIT_SUCCESS);
  
}
 
void control(struct params *motor)
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
        case circle_mode:
            circleCtrl(&param,trig);
            break;
        case line_mode:
            lineCtrl(&param,trig);
            break;
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
      // if(stat1 == ERROR) {
      //   fin();
      //   return;
      // }
			break;
		case simulation:
			break;

		default:
			break;
	}

    RecodeData(FALSE);
	endTask();
	return;

}


void lineCtrl(struct params *param, int trig)
{
	static unsigned long tick=0;
	static double init_pos[7] = {10,20,0,30,40,50,60};
	double desTime;

  	int i;

	if(trig){
		tick = 0;
		for(i=0;i<7;i++) {
			init_pos[i] = cur_j[i].pos;
		}
        desTime = param->desTime;
        linePathInit(init_pos, desTime);
    }
	
	linePathGenerate(des_j, cur_j, tick);

	pdCtrl(des_j, cur_j);
	tick++;

	return;
}


void circleCtrl(struct params *param, int trig)
{
	static unsigned long tick=0;
	static double init_pos[7] = {10,20,0,30,40,50,60};
	double desTime;

  	int i;

	if(trig){
		tick = 0;
		for(i=0;i<7;i++) {
			init_pos[i] = cur_j[i].pos;
		}
        desTime = param->desTime;
        circlePathInit(init_pos, desTime);
    }
	
	circlePathGenerate(des_j, cur_j, tick);

	pdCtrl(des_j, cur_j);
	tick++;

	return;
}


void jointCtrl(struct params *param, int trig)
{
  static unsigned long tick=0;
  static double desPos[7] = {0,0,0,0,0,0,0};
  static double init_pos[7] = {0,0,0,0,0,0,0};
  double desTime;

  int i;

  if(trig){
    tick = 0;
    for(i=0;i<7;i++) {
      desPos[i] = param->desPos[i];
      init_pos[i] = cur_j[i].pos;
    }
    desTime = param->desTime;
    pathInit_j(init_pos, desPos, desTime);
  }

  pathGenerate_j(des_j, tick);

  tick++;
  pdCtrl(des_j, cur_j);

  return;
}


void getCurrentPosition(struct status *Cur_j)
{
	int i;
    double angle[7] = {0., 0., 0., 0., 0., 0., 0.};
    static double pre_j_pos[7] = {0., 0., 0., 0., 0., 0., 0.};
    static double pre_j_vel[7] = {0., 0., 0., 0., 0., 0., 0.};

    /*--- Selection of control mode ---*/

    switch(CtrlMode){
        case operation:
			GetPosition(angle);
            break;
        case simulation:
			for(i=0;i<7;i++) 
				angle[i] = des_j[i].pos;
            break;

        default:
            break;
    }

    /*--- get joint angles ---*/

	for(i=0;i<7;i++) {
		Cur_j[i].pos = angle[i];

		Cur_j[i].vel = (Cur_j[i].pos - pre_j_pos[i]) / TICKS;
		Cur_j[i].acc = (Cur_j[i].vel - pre_j_vel[i]) / TICKS;

		pre_j_pos[i] = Cur_j[i].pos;
		pre_j_vel[i] = Cur_j[i].vel;
	}
}


void linePathInit(double *Start, double Time)
{
	int i;
	double t3, t4, t5;
	double diff;
    double T[4][4];
    double d[3] = {0,100,0};

    kine(Start,T);
    //printf("T=\n");
    //for(i=0;i<3;i++) {
    //    printf("%f    %f    %f   %f\n",T[i][0], T[i][1], T[i][2], T[i][3]);
    //}

    memcpy(lpath.T,T,sizeof(double) * 4*4);
    for(i=0;i<7;i++) lpath.pre_pos[i] = Start[i];
    for(i=0;i<7;i++) lpath.pre_vel[i] = 0;

	t3 = Time * Time * Time;
	t4 = t3 * Time;
	t5 = t4 * Time;

    diff = 1;

    lpath.pos[0] = 0;
    lpath.pos[3] =  10. * diff / t3;
    lpath.pos[4] = -15. * diff / t4;
    lpath.pos[5] =   6. * diff / t5;

    lpath.vel[2] = 3. * lpath.pos[3];
    lpath.vel[3] = 4. * lpath.pos[4];
    lpath.vel[4] = 5. * lpath.pos[5];

    lpath.acc[1] =  6. * lpath.pos[3];
    lpath.acc[2] = 12. * lpath.pos[4];
    lpath.acc[3] = 20. * lpath.pos[5];

    lpath.time = Time;

    for(i=0;i<3;i++) {
        lpath.a[i] = T[i][3];
        lpath.d[i] = d[i];
    }
    printf("lpath.pos[5] = %f\n, lpath.a[0] = %f\n",lpath.pos[5], lpath.a[0]);
}


void linePathGenerate(struct status *Des_j, struct status *Cur_j, unsigned long Time)
{

	int i;
	double t;
    double th[7];

    t = (Time * TICKS);
    t = (t > lpath.time) ? lpath.time : t;

	for(i=0;i<3;i++) {
		lpath.T[i][3] = lpath.a[i] + (lpath.pos[0]
			+ t * t * t * (lpath.pos[3]
					+ t * (lpath.pos[4] + t * lpath.pos[5])))*lpath.d[i];
	}

    invkine(th, lpath.T);
    for(i=0;i<7;i++) {
        Des_j[i].pos = th[i];
        Des_j[i].vel = (Des_j[i].pos - lpath.pre_pos[i])/TICKS;
        Des_j[i].acc = (Des_j[i].vel - lpath.pre_vel[i])/TICKS;

        lpath.pre_pos[i] = Des_j[i].pos;
        lpath.pre_vel[i] = Des_j[i].vel;
    }
}


void circlePathInit(double *Start, double Time)
{
	int i;
	double t3, t4, t5;
	double diff;
    double T[4][4];
    double r = 50;
    double phase = 0;

    kine(Start,T);
    //printf("T=\n");
    //for(i=0;i<3;i++) {
    //    printf("%f    %f    %f   %f\n",T[i][0], T[i][1], T[i][2], T[i][3]);
    //}

    memcpy(cpath.T,T,sizeof(double) * 4*4);
    for(i=0;i<7;i++) cpath.pre_pos[i] = Start[i];
    for(i=0;i<7;i++) cpath.pre_vel[i] = 0;

	t3 = Time * Time * Time;
	t4 = t3 * Time;
	t5 = t4 * Time;

    diff = 2*PI;

    cpath.pos[0] = 0;
    cpath.pos[3] =  10. * diff / t3;
    cpath.pos[4] = -15. * diff / t4;
    cpath.pos[5] =   6. * diff / t5;

    cpath.vel[2] = 3. * cpath.pos[3];
    cpath.vel[3] = 4. * cpath.pos[4];
    cpath.vel[4] = 5. * cpath.pos[5];

    cpath.acc[1] =  6. * cpath.pos[3];
    cpath.acc[2] = 12. * cpath.pos[4];
    cpath.acc[3] = 20. * cpath.pos[5];

    cpath.time = Time;

    cpath.r     = r;
    cpath.phase = phase;

    for(i=0;i<3;i++) cpath.a[i] = cpath.T[i][3];

    //printf("cpath.pos[5] = %f\n, cpath.a[0] = %f\n",cpath.pos[5], cpath.a[0]);
}


void circlePathGenerate(struct status *Des_j, struct status *Cur_j, unsigned long Time)
{

	int i;
	double t;
    double th[7];
    double ith;

    t = (Time * TICKS);
    t = (t > cpath.time) ? cpath.time : t;

    ith = cpath.pos[0] + t * t * t * (cpath.pos[3]
            + t * (cpath.pos[4] + t * cpath.pos[5]));

    cpath.T[0][3] = cpath.a[0];
    cpath.T[1][3] = cpath.a[1] + cpath.r*cos(ith+cpath.phase) - cpath.r*cos(cpath.phase);
    cpath.T[2][3] = cpath.a[2] + cpath.r*sin(ith+cpath.phase) - cpath.r*sin(cpath.phase);

    invkine(th, cpath.T);
    for(i=0;i<7;i++) {
        Des_j[i].pos = th[i];
        Des_j[i].vel = (Des_j[i].pos - cpath.pre_pos[i])/TICKS;
        Des_j[i].acc = (Des_j[i].vel - cpath.pre_vel[i])/TICKS;

        cpath.pre_pos[i] = Des_j[i].pos;
        cpath.pre_vel[i] = Des_j[i].vel;
    }

}


void pathInit_j(double *Start, double *Destination, double Time)
{
	double t3, t4, t5;
	double diff;
	int i;

	t3 = Time * Time * Time;
	t4 = t3 * Time;
	t5 = t4 * Time;


	for(i=0;i<7;i++) {
		diff = Destination[i] - Start[i];

		path_j[i].pos[0] = Start[i];
		path_j[i].pos[3] =  10. * diff / t3;
		path_j[i].pos[4] = -15. * diff / t4;
		path_j[i].pos[5] =   6. * diff / t5;

		path_j[i].vel[2] = 3. * path_j[i].pos[3];
		path_j[i].vel[3] = 4. * path_j[i].pos[4];
		path_j[i].vel[4] = 5. * path_j[i].pos[5];

		path_j[i].acc[1] =  6. * path_j[i].pos[3];
		path_j[i].acc[2] = 12. * path_j[i].pos[4];
		path_j[i].acc[3] = 20. * path_j[i].pos[5];

		path_j[i].time = Time;
	}
}

void pathGenerate_j(struct status *Des_j, unsigned long Time)
{

	int i;
	double t;

	for(i=0;i<7;i++) {
		t = (Time * TICKS);
		t = (t > path_j[i].time) ? path_j[i].time : t;

		Des_j[i].pos = path_j[i].pos[0]
			+ t * t * t * (path_j[i].pos[3]
					+ t * (path_j[i].pos[4] + t * path_j[i].pos[5]));

		Des_j[i].vel = t * t * (path_j[i].vel[2]
				+ t * (path_j[i].vel[3] + t * path_j[i].vel[4]));

		Des_j[i].acc = t * (path_j[i].acc[1] + t * (path_j[i].acc[2]
					+ t * path_j[i].acc[3]));
	}
}


void pdCtrl(struct status *Des_j, struct status *Cur_j)
{
 
	int i;
	double accel[7];
  double th[7];

  for(i=0;i<7;i++) th[i] = Cur_j[i].pos;

  for(i=0;i<7;i++) {
    if(i==5) {
    accel[i] = Des_j[i].acc
       + Kd[i] * (Des_j[i].vel - Cur_j[i].vel)
       + Kp[i] * (Des_j[i].pos - Cur_j[i].pos);

    torque[i] = inertia[i] * accel[i]; // + gcompen(th,i);

    if(torque[i] > max_torque[i])
      torque[i] = max_torque[i];
    else if(torque[i] < - max_torque[i])
      torque[i] = - max_torque[i];
    }
    else {
      torque[i] = 0;
    }
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

int endTask( void )
{
	ctrlEndFlag = 1;
	pthread_mutex_unlock ( &mutex );

	return (EXIT_SUCCESS);
}


void allbrakeoff(void)
{
	AllBrakeOFF(); 
}


void brakeoff(int joint)
{
	BrakeOFF(joint);
}


void Nop(void)
{
	NoTorq();
}


void joint_moveto(double *Angle, double Time)
{
	int i;

	printf("joint_moveto \n");
	for(i=0;i<7;i++) {
		motorp.desPos[i] = Angle[i];  
	}
	motorp.desTime 	 = Time;
	motorp.trig    	 = TRUE;
	motorp.mode    	 = joint_mode;
	printf("destination time= %f\n",Time);
}


void line_moveto(double Time)
{
	int i;

	printf("line_moveto \n");
	for(i=0;i<7;i++) {
		motorp.desPos[i] = 10;  
	}
	motorp.desTime 	 = Time;
	motorp.trig    	 = TRUE;
	motorp.mode    	 = line_mode;
	printf("destination time= %f\n",Time);
}


void circle_moveto(double Time)
{
	int i;

	printf("circle_moveto \n");
	for(i=0;i<7;i++) {
		motorp.desPos[i] = 0;  
	}
	motorp.desTime 	 = Time;
	motorp.trig    	 = TRUE;
	motorp.mode    	 = circle_mode;
	printf("destination time= %f\n",Time);
}


void All_OFFBrake(void)
{
  
	motorp.mode = allbrakeoff_mode;
}


void
OFFBrake(void)
{
  
	motorp.mode = brakeoff_mode;
	motorp.joint = brakeoff_joint;
}

void ONBrake(void)
{  
	motorp.mode = nop;
}


void fin(void)
{
  fclose(fp);

  ctrltrig = 0;
  ONBrake();
  arcFin();
  timer_delete(timerid);
  RecodeData(TRUE);
}


void RecodeData(int endFlag)
{
	int i,j,k,l;
	static int tick=0, mt;
    static int min;

    int n=100,m=11;
    static double **mindata, *base_mindata;
    static double **logdata[100000];

    double th[7];
    double reco_volt[7];
    double T[4][4];

    if(!endFlag) {

        if(tick%1000 == 0) {
          AGetCmdVolt(reco_volt);

          printf("t= %.1f:",tick*TICKS);
          for(l=0;l<7;l++)
            printf(" [%d]%f,", l,cur_j[l].pos*RAD2DEG);
          printf("\n");

          printf("t= %.1f:",tick*TICKS);
          for(l=0;l<7;l++)
            printf(" [%d]%f,", l,reco_volt[l]);
          printf(" [V]\n");
        }

        if(tick % n  == 0) {
            mt  = 0;
            min = tick / n;

            mindata         = malloc(sizeof(double *) * n);
            base_mindata    = malloc(sizeof(double) * n * m);
            for(i=0;i<n;i++) mindata[i] = base_mindata + i * m;

            logdata[min]    = mindata;
        }

        i=0;
        mindata[mt][i++] = (double)tick*TICKS;

        for(j=0;j<7;j++) {
            th[j] = cur_j[j].pos;
            mindata[mt][i++] = cur_j[j].pos*RAD2DEG;
        }

        kine(th,T);
        for(j=0;j<3;j++) mindata[mt][i++] = T[j][3];
        //for(j=0;j<2;j++) mindata[mt][i++] = des_j[j].vel;
        //for(j=0;j<1;j++) mindata[mt][i++] = des_j[j].acc;

        tick++;
        mt++;

    } else {

        printf("Start recoding data\n");

        //open file
        if ((fp = fopen(filename, "w")) == NULL) {
            fprintf(stderr, "faile open file \n");
            return;
        }

        fprintf(fp,"time,ang1,ang2,ang3,ang4,ang5,ang6,ang7,posx,posy,poz\n");
        
        for(i=0;i<min;i++) {
            for(j=0;j<n;j++) {
                for(k=0;k<m;k++) {
                    fprintf(fp,"%f,",logdata[i][j][k]);
                }
                fprintf(fp,"\n");
            }
        }

        for(i=0;i<mt;i++) {
            for(j=0;j<m;j++) {
                fprintf(fp,"%f,",logdata[min][i][j]);
            }
            fprintf(fp,"\n");
        }
        
        fclose(fp);
        printf("End recoding data\n");

        free(base_mindata);
        free(mindata);
    }
}


int kine(double th[7],double T[4][4])
{
    double c1 = cos(th[0]), c2 = cos(th[1]), c3 = cos(th[2]), c4 = cos(th[3]),
           c5 = cos(th[4]), c6 = cos(th[5]), c7 = cos(th[6]);
    double s1 = sin(th[0]), s2 = sin(th[1]), s3 = sin(th[2]), s4 = sin(th[3]),
           s5 = sin(th[4]), s6 = sin(th[5]), s7 = sin(th[6]);

    if(fabs(th[0]) > 177* DEG2RAD
        || fabs(th[1]) > 94* DEG2RAD
        || fabs(th[2]) > 174* DEG2RAD
        || fabs(th[3]) > 137* DEG2RAD
        || fabs(th[4]) > 255* DEG2RAD
        || fabs(th[5]) > 165* DEG2RAD
        || fabs(th[6]) > 255* DEG2RAD ) return 1;


    T[0][0] = c1*(c2*(c3*(c4*(c5*c6*c7-s5*s7)-c7*s4*s6)-s3*(c5*s7+c6*c7*s5))+s2*(-s4*(c5*c6*c7-s5*s7)-c4*c7*s6))-s1*(s3*(c4*(c5*c6*c7-s5*s7)-c7*s4*s6)+c3*(c5*s7+c6*c7*s5));
    T[1][0] = s1*(c2*(c3*(c4*(c5*c6*c7-s5*s7)-c7*s4*s6)-s3*(c5*s7+c6*c7*s5))+s2*(-s4*(c5*c6*c7-s5*s7)-c4*c7*s6))+c1*(s3*(c4*(c5*c6*c7-s5*s7)-c7*s4*s6)+c3*(c5*s7+c6*c7*s5));
    T[2][0] = c2*(-s4*(c5*c6*c7-s5*s7)-c4*c7*s6)-s2*(c3*(c4*(c5*c6*c7-s5*s7)-c7*s4*s6)-s3*(c5*s7+c6*c7*s5));
    T[3][0] = 0;
        
    T[0][1] = c1*(c2*(c3*(c4*(-c5*c6*s7-c7*s5)+s4*s6*s7)-s3*(c5*c7-c6*s5*s7))+s2*(c4*s6*s7-s4*(-c5*c6*s7-c7*s5)))-s1*(s3*(c4*(-c5*c6*s7-c7*s5)+s4*s6*s7)+c3*(c5*c7-c6*s5*s7));
    T[1][1] = s1*(c2*(c3*(c4*(-c5*c6*s7-c7*s5)+s4*s6*s7)-s3*(c5*c7-c6*s5*s7))+s2*(c4*s6*s7-s4*(-c5*c6*s7-c7*s5)))+c1*(s3*(c4*(-c5*c6*s7-c7*s5)+s4*s6*s7)+c3*(c5*c7-c6*s5*s7));
    T[2][1] = c2*(c4*s6*s7-s4*(-c5*c6*s7-c7*s5))-s2*(c3*(c4*(-c5*c6*s7-c7*s5)+s4*s6*s7)-s3*(c5*c7-c6*s5*s7));
    T[3][1] = 0;
        
    T[0][2] = c1*(c2*(c3*(c4*c5*s6+c6*s4)-s3*s5*s6)+s2*(c4*c6-c5*s4*s6))-s1*(s3*(c4*c5*s6+c6*s4)+c3*s5*s6);
    T[1][2] = s1*(c2*(c3*(c4*c5*s6+c6*s4)-s3*s5*s6)+s2*(c4*c6-c5*s4*s6))+c1*(s3*(c4*c5*s6+c6*s4)+c3*s5*s6);
    T[2][2] = c2*(c4*c6-c5*s4*s6)-s2*(c3*(c4*c5*s6+c6*s4)-s3*s5*s6);
    T[3][2] = 0;
        
    T[0][3] = c1*(c2*(c3*(70*c4*c5*s6+(70*c6+480)*s4)-70*s3*s5*s6)+s2*(-70*c5*s4*s6+c4*(70*c6+480)+450))-s1*(s3*(70*c4*c5*s6+(70*c6+480)*s4)+70*c3*s5*s6);
    T[1][3] = s1*(c2*(c3*(70*c4*c5*s6+(70*c6+480)*s4)-70*s3*s5*s6)+s2*(-70*c5*s4*s6+c4*(70*c6+480)+450))+c1*(s3*(70*c4*c5*s6+(70*c6+480)*s4)+70*c3*s5*s6);
    T[2][3] = -s2*(c3*(70*c4*c5*s6+(70*c6+480)*s4)-70*s3*s5*s6)+c2*(-70*c5*s4*s6+c4*(70*c6+480)+450)+317;
    T[3][3] = 1;

    return 0;
}


int invkine(double th[7], double T[4][4]) {
    double tmp;
    double psx,psy,psz;
    double c1,c4,c12,c5,s1,s4,s12,s5;
    double nx,ny,tx,ty,bx,by,bz;

    th[2] = 0;

    psx = T[0][3] - 70*T[0][2];
    psy = T[1][3] - 70*T[1][2];
    psz = T[2][3] - 70*T[2][2];

    th[0] = atan2(psy,psx);
    if(th[0] > 177*DEG2RAD)         th[0] = th[0] - 2*PI;
    else if(th[0] < -177*DEG2RAD)   th[0] = th[0] + 2*PI;

    tmp = (psx*psx+psy*psy+pow((-psz+317),2)-450*450-480*480)/(450*480*2);
    th[3] = 2*atan2(sqrt(1-tmp),sqrt(1+tmp));
    if(th[3] > 137*DEG2RAD)         th[3] = th[3] - 2*PI;
    else if(th[3] < -137*DEG2RAD)   th[3] = th[3] + 2*PI;

    c1 = cos(th[0]); c4 = cos(th[3]);
    s1 = sin(th[0]); s4 = sin(th[3]);
    tmp = ((psx*c1+psy*s1)*(480*c4+450)+(-psz+317)*480*s4)/(pow((psx*c1+psy*s1),2)+pow((-psz+317),2));
    th[1] = 2*atan2(1-sqrt(1-tmp*tmp),tmp);
    if(th[1] > 94*DEG2RAD)          th[1] = th[1] - 2*PI;
    else if(th[1] < -94*DEG2RAD)    th[1] = th[1] + 2*PI;

    c12 = cos(th[0]+th[1]); s12 = sin(th[0]+th[1]);
    nx = -T[2][0]*sin(th[1]+th[3])+T[1][0]*sin(th[0])*cos(th[1]+th[3])+T[0][0]*cos(th[0])*cos(th[1]+th[3]);
    ny = T[0][0]*((sin(2*th[3]+2*th[1]+th[0])-sin(2*th[3]+2*th[1]-th[0]))/4-sin(2*th[3]+2*th[1]+th[0])/4+sin(2*th[3]+2*th[1]-th[0])/4-sin(th[0]))+T[1][0]*(cos(2*th[3]+2*th[1]+th[0])/4+(-cos(2*th[3]+2*th[1]+th[0])-cos(2*th[3]+2*th[1]-th[0]))/4+cos(2*th[3]+2*th[1]-th[0])/4+cos(th[0]));
    tx = -T[2][1]*sin(th[3]+th[1])+T[1][1]*sin(th[0])*cos(th[3]+th[1])+T[0][1]*cos(th[0])*cos(th[3]+th[1]);
    ty = T[0][1]*((sin(2*th[3]+2*th[1]+th[0])-sin(2*th[3]+2*th[1]-th[0]))/4-sin(2*th[3]+2*th[1]+th[0])/4+sin(2*th[3]+2*th[1]-th[0])/4-sin(th[0]))+T[1][1]*(cos(2*th[3]+2*th[1]+th[0])/4+(-cos(2*th[3]+2*th[1]+th[0])-cos(2*th[3]+2*th[1]-th[0]))/4+cos(2*th[3]+2*th[1]-th[0])/4+cos(th[0]));
    bx = -T[2][2]*sin(th[1]+th[3])+T[1][2]*sin(th[0])*cos(th[1]+th[3])+T[0][2]*cos(th[0])*cos(th[3]+th[1]);
    by = T[0][2]*((sin(2*th[3]+2*th[1]+th[0])-sin(2*th[3]+2*th[1]-th[0]))/4-sin(2*th[3]+2*th[1]+th[0])/4+sin(2*th[3]+2*th[1]-th[0])/4-sin(th[0]))+T[1][2]*(cos(2*th[3]+2*th[1]+th[0])/4+(-cos(2*th[3]+2*th[1]+th[0])-cos(2*th[3]+2*th[1]-th[0]))/4+cos(2*th[3]+2*th[1]-th[0])/4+cos(th[0]));
    bz = T[1][2]*sin(th[0])*sin(th[3]+th[1])+T[0][2]*cos(th[0])*sin(th[3]+th[1])+T[2][2]*cos(th[3]+th[1]);

    th[4] = atan2(by,bx);
    if(th[4] > 255*DEG2RAD)         th[4] = th[4] - 2*PI;
    else if(th[4] < -255*DEG2RAD)   th[4] = th[4] + 2*PI;

    c5 = cos(th[4]); s5 = sin(th[4]);
    th[5] = atan2((bx*c5+by*s5),bz);
    if(th[5] > 165*DEG2RAD)         th[5] = th[5] - 2*PI;
    else if(th[5] < -165*DEG2RAD)   th[5] = th[5] + 2*PI;

    th[6] = atan2((-nx*s5+ny*c5),(-tx*s5+ty*c5));
    if(th[6] > 255*DEG2RAD)         th[6] = th[6] - 2*PI;
    else if(th[6] < -255*DEG2RAD)   th[6] = th[6] + 2*PI;

    return 0;
}


double gcompen(double th[7],int dth)
{
    double g = 9.81;
    double m[13] = {6.88, 4.92, 2.41, 6.87, 3.89, 0.62, 2.35, 3.29, 1.57, 0.47, 2.61, 2.07, 1.05};
    double c[13] = {0.091, 0.105, 0.2015, 0.274, 0.175, 0.205, 0.368, 0.450, 0.150, 0.211, 0.396, 0.438, 0.022}; // [ mm ]
    double c1=cos(th[1]),c2=cos(th[2]),c3=cos(th[3]),c4=cos(th[4]),c5=cos(th[5]);
    double s1=sin(th[1]),s2=sin(th[2]),s3=sin(th[3]),s4=sin(th[4]),s5=sin(th[5]);
    double gcom=0;

    switch(dth) {
        case 0:
            gcom = 0;
            break;
        case 1:
            gcom = -g * m[8] * c[8]*c1*c2*s3-g * m[8] * s1*(c[8]*c3+0.45)-g * m[7] * c[7]*s1-g * m[6] * c[6]*s1-g * m[5] * c[5]*s1-g * m[4] * c[4]*s1-g * m[12] * c1*(c2*(c[12]*c3*c4*s5+s3*(c[12]*c5+0.48))-c[12]*s2*s4*s5)-g * m[12] * s1*(-c[12]*s3*c4*s5+c3*(c[12]*c5+0.48)+0.45)-g * m[11] * c[11]*c1*c2*s3-g * m[11] * s1*(c[11]*c3+0.45)-g * m[10] * c[10]*c1*c2*s3-g * m[10] * s1*(c[10]*c3+0.45)-g * m[9] * c[9]*c1*c2*s3-g * m[9] * s1*(c[9]*c3+0.45);
            break;
        case 2:
            gcom = g * m[8] * c[8]*s1*s2*s3-g * m[12] * s1*(-s2*(c[12]*c3*c4*s5+s3*(c[12]*c5+0.48))-c[12]*c2*s4*s5)+g * m[11] * c[11]*s1*s2*s3+g * m[10] * c[10]*s1*s2*s3+g * m[9] * c[9]*s1*s2*s3;
            break;
        case 3:
            gcom = -g * m[8] * c[8]*c1*s3-g * m[8] * c[8]*s1*c2*c3-g * m[12] * s1*c2*(c3*(c[12]*c5+0.48)-c[12]*s3*c4*s5)+g * m[12] * c1*(-c[12]*c3*c4*s5-s3*(c[12]*c5+0.48))-g * m[11] * c[11]*c1*s3-g * m[11] * c[11]*s1*c2*c3-g * m[10] * c[10]*c1*s3-g * m[10] * c[10]*s1*c2*c3-g * m[9] * c[9]*c1*s3-g * m[9] * c[9]*s1*c2*c3;
            break;
        case 4:
            gcom = g * m[12] * c[12]*c1*s3*s4*s5-g * m[12] * s1*(-c[12]*c2*c3*s4*s5-c[12]*s2*c4*s5);
            break;
        case 5:
            gcom = g * m[12] * c1*(-c[12]*c3*s5-c[12]*s3*c4*c5)-g * m[12] * s1*(c2*(c[12]*c3*c4*c5-c[12]*s3*s5)-c[12]*s2*s4*c5);
            break;
        case 6:
            gcom = 0;
            break;
    }

    return gcom;
}


int main(void)
{
    cp(NULL, NULL);      
    return(OK);

}



