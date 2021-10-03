#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PI 3.14159265
#define val PI/180
#define cons 180/PI

void multiply(float mat1[4][4], float mat2[4][4], float res[4][4])
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

void Tranf_Matrix(float arr[4][4],int alpha, int a, int d, float theta)
{
    arr[0][0] = cos(theta*val);
    arr[0][1] = -sin(theta*val)*cos(alpha*val);
    arr[0][2] = sin(theta*val)*sin(alpha*val);
    arr[0][3] = a*cos(theta*val);

    arr[1][0] = sin(theta*val);
    arr[1][1] = cos(theta*val)*cos(alpha*val);
    arr[1][2] = -cos(theta*val)*sin(alpha*val);
    arr[1][3] = a*sin(theta*val);

    arr[2][0] = 0;
    arr[2][1] = sin(alpha*val);
    arr[2][2] = cos(alpha*val);
    arr[2][3] = d;

    arr[3][0] = 0;
    arr[3][1] = 0;
    arr[3][2] = 0;
    arr[3][3] = 1;

    return 0;

}

void DH_params(float arr[7][4])
{
    arr[0][0] = 90;
    arr[0][1] = 100;
    arr[0][2] = 320;
    printf("Enter the value of Theta_1:");
    scanf("%f", &arr[0][3]);

    arr[1][0] = -90;
    arr[1][1] = 0;
    arr[1][2] = 0;
    printf("Enter the value of Theta_2:");
    scanf("%f", &arr[1][3]);

    arr[2][0] = 90;
    arr[2][1] = 0;
    arr[2][2] = 400;
    arr[2][3] = 0;

    arr[3][0] = -90;
    arr[3][1] = 0;
    arr[3][2] = 0;
    printf("Enter the value of Theta_4:");
    scanf("%f", &arr[3][3]);

    arr[4][0] = 90;
    arr[4][1] = 0;
    arr[4][2] = 350;
    printf("Enter the value of Theta_5:");
    scanf("%f", &arr[4][3]);

    arr[5][0] = -90;
    arr[5][1] = 0;
    arr[5][2] = 0;
    printf("Enter the value of Theta_6:");
    scanf("%f", &arr[5][3]);

    arr[6][0] = 0;
    arr[6][1] = 0;
    arr[6][2] = 65;
    printf("Enter the value of Theta_7:");
    scanf("%f", &arr[6][3]);

    return 0;

}

void comp_A (float A[7][4][4], float A_res[4][4])
{

    float A_dummy[4][4]; /*Storing value of the current result of matrix multiplication*/
    /*Storing the initial value of A_dummy which is A[0]*/
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            A_dummy[i][j]=A[0][i][j];
        }
    }

    /*Multiplying matrix consecutively*/
    for(int i=0; i<6;i++)
    {
        multiply(A_dummy,A[i+1],A_res);
        /*Passing value of the current result to A dummy*/
        for(int j=0;j<4;j++)
        {
            for(int k=0;k<4;k++)
            {
                /*printf("%f ", A_res[j][k]);*/
                A_dummy[j][k]=A_res[j][k];

            }
            /*printf("\n");*/
        }
        /*printf("\n");*/
    }
}

void Calc_A(float B[4][4], float theta[7])
{
    float t1 = val*theta[0];
    float t2 = val*theta[1];
    float t4 = val*theta[3];
    float t5 = val*theta[4];
    float t6 = val*theta[5];
    float t7 = val*theta[6];

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

    return 0;
}

void Error(float A[4][4],float B[4][4], float E[4][4])
{
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            E[i][j]=A[i][j]-B[i][j];
            /*printf("%f ", E[i][j]);*/
        }
        /*printf("\n");*/
    }
    /*printf("\n");*/

    return 0;
}

void Inverse(float A[4][4], float Ainv[4][4])
{
   for(int i=0; i<3; i++)
   {
       for(int j=0; j<3; j++)
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

   return 0;
}

void Print(float A[4][4])
{
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            printf("%f ", A[i][j]);
        }
        printf("\n");
    }
    printf("\n");

    return 0;
}

void PassInv (float A[4][4][4], float Ainv[4][4][4]) /*Calculate and pass the inverse value of transformation matrix*/
{
    for(int i=0;i<4;i++)
    {
        Inverse(A[i], Ainv[i]);
    }
    return 0;
}

void InverseKin1 (float F[4][4], float theta[7])
{
    float t1 ;
    float t2 ;
    float t3 = 0;
    float t4 ;

    float gy = -65*F[1][2]+F[1][3];
    float gx = -65*F[0][2]+F[0][3];
    float gz = -65*F[2][2]+F[2][3];
    t1 = cons*atan2(gy, gx);
    if(t1<0)
    {
        t1 = t1+180;
    }

    float hx = cos(t1*val)*gx + sin(t1*val)*gy - 100;
    float hy = gz-320;
    float hz = sin(t1*val)*gx - cos(t1*val)*gy;
    t4 = cons*acos((hx*hx + hy*hy + hz*hz - 350*350 - 400*400)/(2*350*400));

    float k1 = 350*cos(t4*val)+400;
    float k2 = 350*sin(t4*val);
    t2 = 2*cons*atan2(-k2 + sqrt(k2*k2 + k1*k1 - hy*hy),hy+k1);
    if(t2<0)
    {
        t2 = 360+t2;
    }
    theta[0]=t1;
    theta[1]=t2;
    theta[2]=t3;
    theta[3]=t4;

    return 0;
}

void InverseKin2(float F[4][4], float Ainv[3][4][4], float theta[7])
{
    float t5;
    float t6;
    float t7;

    float J[4][4];
    float Jdummy[4][4];
    for(int i=0;i<4;i++) /*Passing inverse T.matrix to Jdummy*/
    {
        for(int j=0;j<4;j++)
        {
            Jdummy[i][j]=Ainv[3][i][j];
        }
    }

    /*Multiplying matrix consecutively*/
    for(int i=0; i<3;i++)
    {
        multiply(Jdummy,Ainv[2-i],J);
        /*Passing value of the current result to J dummy*/
        for(int j=0;j<4;j++)
        {
            for(int k=0;k<4;k++)
            {
                Jdummy[j][k]=J[j][k];
            }
        }
    }
    multiply(Jdummy,F,J);
    t5 = cons*atan2(J[1][2], J[0][2]);
    if(t5<0)
    {
        t5 = t5+180;
    }
    t6 = cons*atan2(-J[0][2]*cos(val*t5) - J[1][2]*sin(val*t5), J[2][2]);
    if(t6<0)
    {
        t6 = t6+180;
    }
    t7 = cons*atan2(J[0][0]*sin(val*t5) - J[1][0]*cos(val*t5) , J[0][1]*sin(val*t5) - J[1][1]*cos(val*t5) );
    if(t7<0)
    {
        t7 = t7+180;
    }

    theta[4] = t5;
    theta[5] = t6;
    theta[6] = t7;
    return 0;
}
int main()
{
    float DH[7][4];
    DH_params(DH); /*store value of DH Parameters*/

    float theta[7];
    for(int i = 0;i<7;i++)
    {
        theta[i]=DH[i][3];
    }



    float A[7][4][4]; /*Matrix for forward kinematics*/
    float A_res[4][4]; /*Result of matrix multiplication*/
    float B[4][4]; /*result of hand calculation*/
    float E[4][4]; /*error*/

    float C[4][4][4]; /*Matrix to store T.matrix obtained from Inverse kinematics*/
    float Ainv[4][4][4]; /*set of inverse of transformation matrix*/
    float thetaInv[7];

    /*store value to each transformation matrix*/
    for(int i = 0;i<7;i++)
    {
        Tranf_Matrix(A[i],DH[i][0],DH[i][1],DH[i][2],DH[i][3]);
    }


    comp_A(A,A_res);
    Calc_A(B, theta);
    Error(A_res, B, E);


    InverseKin1(A_res,thetaInv);

    for(int i = 0;i<4;i++)
    {
        Tranf_Matrix(C[i],DH[i][0],DH[i][1],DH[i][2],thetaInv[i]);
    }

    PassInv(C, Ainv);



    InverseKin2(A_res,Ainv,thetaInv);

    for(int i=0;i<7;i++)
    {
        printf("Inverse Kinematics Theta %d = %f\n", i+1, thetaInv[i]);
    }








    return 0;
}
