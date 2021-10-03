#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define val PI/180
#define cons 180/PI
#define PI 3.1415926535

void multiply(double mat1[4][4], double mat2[4][4], double res[4][4])
{
    int i, j, k;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            res[i][j] = 0;
            for (k = 0; k < 4; k++)
                res[i][j] += mat1[i][k]*mat2[k][j];
        }
    }
}

void Tranf_Matrix(double arr[4][4],double alpha, int a, int d, double theta)
{
    arr[0][0] = cos(theta);
    arr[0][1] = -sin(theta)*cos(alpha);
    arr[0][2] = sin(theta)*sin(alpha);
    arr[0][3] = a*cos(theta);

    arr[1][0] = sin(theta);
    arr[1][1] = cos(theta)*cos(alpha);
    arr[1][2] = -cos(theta)*sin(alpha);
    arr[1][3] = a*sin(theta);

    arr[2][0] = 0;
    arr[2][1] = sin(alpha);
    arr[2][2] = cos(alpha);
    arr[2][3] = d;

    arr[3][0] = 0;
    arr[3][1] = 0;
    arr[3][2] = 0;
    arr[3][3] = 1;
}

void DH_params(double arr[7][4])
{
    arr[0][0] = 90*val;
    arr[0][1] = 100;
    arr[0][2] = 320;


    arr[1][0] = -90*val;
    arr[1][1] = 0;
    arr[1][2] = 0;

    arr[2][0] = 90*val;
    arr[2][1] = 0;
    arr[2][2] = 400;
    arr[2][3] = 0;

    arr[3][0] = -90*val;
    arr[3][1] = 0;
    arr[3][2] = 0;

    arr[4][0] = 90*val;
    arr[4][1] = 0;
    arr[4][2] = 350;

    arr[5][0] = -90*val;
    arr[5][1] = 0;
    arr[5][2] = 0;

    arr[6][0] = 0;
    arr[6][1] = 0;
    arr[6][2] = 65;

}

void comp_A (double A[7][4][4], double A_res[4][4])
{
	int i, j, k;
    double A_dummy[4][4]; /*Storing value of the current result of matrix multiplication*/
    /*Storing the initial value of A_dummy which is A[0]*/
    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            A_dummy[i][j]=A[0][i][j];
        }
    }

    /*Multiplying matrix consecutively*/
    for(i=0; i<6;i++)
    {
        multiply(A_dummy,A[i+1],A_res);
        /*Passing value of the current result to A dummy*/
        for(j=0;j<4;j++)
        {
            for(k=0;k<4;k++)
            {
                /*printf("%f ", A_res[j][k]);*/
                A_dummy[j][k]=A_res[j][k];

            }
            /*printf("\n");*/
        }
        /*printf("\n");*/
    }
}

void Calc_A(double B[4][4], double theta[7])
{
    double t1 = val*theta[0];
    double t2 = val*theta[1];
    double t4 = val*theta[3];
    double t5 = val*theta[4];
    double t6 = val*theta[5];
    double t7 = val*theta[6];

    B[0][0] = cos(t1)*cos(t2+t4)*(cos(t5)*cos(t6)*cos(t7)-sin(t5)*sin(t7)) -sin(t1)*(sin(t5)*cos(t6)*cos(t7)+cos(t5)*sin(t7)) -cos(t1)*sin(t2+t4)*sin(t6)*cos(t7);
    B[0][1] = -cos(t1)*cos(t2+t4)*(cos(t5)*cos(t6)*sin(t7)+sin(t5)*cos(t7)) +sin(t1)*(sin(t5)*cos(t6)*sin(t7)-cos(t5)*cos(t7)) +cos(t1)*sin(t2+t4)*sin(t6)*sin(t7);
    B[0][2] = -cos(t1)*cos(t2+t4)*cos(t5)*sin(t6) +sin(t1)*sin(t5)*sin(t6) -cos(t1)*sin(t2+t4)*cos(t6);
    B[0][3] = cos(t1)*cos(t2+t4)*(-65*cos(t5)*sin(t6)) +sin(t1)*(65*sin(t5)*sin(t6)) -cos(t1)*sin(t2+t4)*(65*cos(t6)+350) -400*cos(t1)*sin(t2)+100*cos(t1);

    B[1][0] = sin(t1)*cos(t2+t4)*(cos(t5)*cos(t6)*cos(t7)-sin(t5)*sin(t7)) +cos(t1)*(sin(t5)*cos(t6)*cos(t7)+cos(t5)*sin(t7)) -sin(t1)*sin(t2+t4)*sin(t6)*cos(t7);
    B[1][1] = -sin(t1)*cos(t2+t4)*(cos(t5)*cos(t6)*sin(t7)+sin(t5)*cos(t7)) +cos(t1)*(-sin(t5)*cos(t6)*sin(t7)+cos(t5)*cos(t7)) +sin(t1)*sin(t2+t4)*sin(t6)*sin(t7);
    B[1][2] = -sin(t1)*cos(t2+t4)*cos(t5)*sin(t6) -cos(t1)*sin(t5)*sin(t6) -sin(t1)*sin(t2+t4)*cos(t6);
    B[1][3] = sin(t1)*cos(t2+t4)*(-65*cos(t5)*sin(t6)) -cos(t1)*(65*sin(t5)*sin(t6)) -sin(t1)*sin(t2+t4)*(65*cos(t6)+350) -400*sin(t1)*sin(t2)+100*sin(t1);

    B[2][0] = sin(t2+t4)*(cos(t5)*cos(t6)*cos(t7)-sin(t5)*sin(t7)) +cos(t2+t4)*sin(t6)*cos(t7);
    B[2][1] = -sin(t2+t4)*(cos(t5)*cos(t6)*sin(t7)+sin(t5)*cos(t7)) -cos(t2+t4)*sin(t6)*sin(t7);
    B[2][2] = -sin(t2+t4)*cos(t5)*cos(t6) + cos(t2+t4)*cos(t6);
    B[2][3] = -65*sin(t2+t4)*cos(t5)*sin(t6) + 65*cos(t2+t4)*cos(t6) + 350*cos(t2+t4) +400*cos(t2)+320;

    B[3][0] = 0;
    B[3][1] = 0;
    B[3][2] = 0;
    B[3][3] = 1;
}

void Error(double A[4][4],double B[4][4], double E[4][4])
{
	int i, j;
	for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            E[i][j]=A[i][j]-B[i][j];
            /*printf("%f ", E[i][j]);*/
        }
        /*printf("\n");*/
    }
    /*printf("\n");*/
}

void Inverse(double A[4][4], double Ainv[4][4])
{
    int i,j;
   for(i=0; i<3; i++)
   {
       for(j=0; j<3; j++)
       {
           Ainv[i][j] = A[j][i];
       }
   }

   Ainv[3][0] = 0;
   Ainv[3][1] = 0;
   Ainv[3][2] = 0;
   Ainv[3][3] = 1;

   Ainv[0][3] = -(Ainv[0][0]*A[0][3] + Ainv[0][1]*A[1][3] + Ainv[0][2]*A[2][3]);
   Ainv[1][3] = -(Ainv[1][0]*A[0][3] + Ainv[1][1]*A[1][3] + Ainv[1][2]*A[2][3]);
   Ainv[2][3] = -(Ainv[2][0]*A[0][3] + Ainv[2][1]*A[1][3] + Ainv[2][2]*A[2][3]);
}

void Print(double A[4][4])
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

void PassInv (double A[4][4][4], double Ainv[4][4][4]) /*Calculate and pass the inverse value of transformation matrix*/
{
	int i;
	for(i=0;i<4;i++)
    {
        Inverse(A[i], Ainv[i]);
    }
}

void InverseKin1 (double F[4][4], double theta[7])
{
    double t1 ;
    double t2 ;
    double t3 = 0;
    double t4 ;

    double gy = -65*F[1][2]+F[1][3];
    double gx = -65*F[0][2]+F[0][3];
    double gz = -65*F[2][2]+F[2][3];
    t1 = atan2(gy, gx);

    double hx = cos(t1)*gx + sin(t1)*gy - 100;
    double hy = gz-320;

    double k = (hx*hx + hy*hy - 350*350 - 400*400)/(2*350*400);
    t4 = 2*atan2(-sqrt(1-k),sqrt(1+k));
    /*printf("%lf, %lf, %lf\n", hx, gx, k);*/

    double k1 = 350*cos(t4)+400;
    double k2 = 350*sin(t4);
    double p1 = -k2 - sqrt(k2*k2 + k1*k1 - hy*hy);
    double p2 = hy+k1;
    t2 = 2*atan2(p1, p2);
    /*printf("tan(t2/2) = %lf\n", (p1/p2));*/
    theta[0]=t1;
    theta[1]=t2;
    theta[2]=t3;
    theta[3]=t4;
}

void InverseKin2(double F[4][4], double Ainv[3][4][4], double theta[7])
{
    double t5;
    double t6;
    double t7;
	int i, j, k;

    double J[4][4];
    double Jdummy[4][4];
    for(i=0;i<4;i++) /*Passing inverse T.matrix to Jdummy*/
    {
        for(j=0;j<4;j++)
        {
            Jdummy[i][j]=Ainv[3][i][j];
        }
    }

    /*Multiplying matrix consecutively*/
    for(i=0; i<3;i++)
    {
        multiply(Jdummy,Ainv[2-i],J);
        /*Passing value of the current result to J dummy*/
        for(j=0;j<4;j++)
        {
            for(k=0;k<4;k++)
            {
                Jdummy[j][k]=J[j][k];
            }
        }
    }
    multiply(Jdummy,F,J);
    t5 = atan2(J[1][2], J[0][2]);
    /*printf("tan(t5) = %lf\n", (J[1][2]/J[0][2]));*/

    t6 = atan2(-J[0][2]*cos(t5) - J[1][2]*sin(t5), J[2][2]);
    /*printf("tan(t6) = %lf\n", (-J[0][2]*cos(val*t5) - J[1][2]*sin(val*t5))/J[2][2]);*/

    t7 = atan2(-J[0][0]*sin(t5) + J[1][0]*cos(t5) , -J[0][1]*sin(t5) + J[1][1]*cos(t5) );
   /* printf("tan(t7) = %lf\n", (J[0][0]*sin(val*t5) - J[1][0]*cos(val*t5))/(J[0][1]*sin(val*t5) - J[1][1]*cos(val*t5)) );*/

    theta[4] = t5;
    theta[5] = t6;
    theta[6] = t7;
}

void FK (double theta[7], double T[4][4])
{
    double DH[7][4];
	int i;
    DH_params(DH);

    for(i = 0;i<7;i++)
    {
        DH[i][3]=theta[i];
    }

     double A[7][4][4]; /*Matrix for forward kinematics*/

    /*store value to each transformation matrix*/
      for(i = 0;i<7;i++)
    {
        Tranf_Matrix(A[i],DH[i][0],DH[i][1],DH[i][2],DH[i][3]);
    }

    comp_A(A,T); /*compute the forward kinematics*/

}

void IKneg (double theta[7], double T[4][4])
{
    double C[4][4][4]; /*Matrix to store T.matrix obtained from Inverse kinematics*/
    double Ainv[4][4][4]; /*set of inverse of transformation matrix*/
    double DH[7][4];
    DH_params(DH);
	int i;

    InverseKin1(T,theta); /*calculate the first three angke*/

    for(i = 0;i<4;i++)
    {
        Tranf_Matrix(C[i],DH[i][0],DH[i][1],DH[i][2],theta[i]);
    }

    PassInv(C, Ainv); /*calculate the inverse of transformation matrix*/

    InverseKin2(T,Ainv,theta);
}

int main()
{
    double theta[7];
    double T[4][4];
    double B[4][4];
    double thetaInv[7];

    for(int i=0; i<7;i++)
    {
        printf("Enter the value of Theta_%d:", i+1);
        scanf("%lf", &theta[i]);
    }
    for(int i=0;i<7;i++)
    {
        theta[i]=theta[i]*val;
        printf("%lf\n", theta[i]);
    }

    FK(theta,T);
    Print(T);
    IK(thetaInv,T);

    for(int i=0;i<7;i++)
    {
        printf("Inverse Kinematics Theta %d = %f\n", i+1, cons*thetaInv[i]);
    }

    return 0;
}
