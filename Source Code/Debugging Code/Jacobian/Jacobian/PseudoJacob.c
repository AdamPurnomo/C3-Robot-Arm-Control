
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "kinematics-Test.c"

extern void DH_params(double arr[7][4]);

void Inv3x3 (double a[3][3], double ainv[3][3])
{
    int i,j;
    double determinant=0;
    for(i=0;i<3;i++)
    {
        determinant = determinant + (a[0][i]*(a[1][(i+1)%3]*a[2][(i+2)%3] - a[1][(i+2)%3]*a[2][(i+1)%3]));
    }
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            ainv[i][j] = ((a[(i+1)%3][(j+1)%3] * a[(i+2)%3][(j+2)%3]) - (a[(i+1)%3][(j+2)%3]*a[(i+2)%3][(j+1)%3]))/ determinant;
        }
    }
}

void multiplyG (int row1, int col1, double mat1[row1][col1], int row2, int col2, double mat2[row2][col2], double res[row1][col2])
{
    int i, j, k;
    for (i = 0; i < row1; i++)
    {
        for (j = 0; j < col2; j++)
        {
            res[i][j] = 0;
            for (k = 0; k < col1; k++)
                res[i][j] += mat1[i][k]*mat2[k][j];
        }
    }
}

void Jac_Trans(double J[3][7], double Jt[7][3])
{
    int i,j;
    for(i=0; i<7; i++)
        {
            for(j=0; j<3; j++)
            {
                Jt[i][j] = J[j][i];
            }
        }
}

void crossProduct(double vect_A[3][1], double vect_B[3][1], double cross_P[3][1])
{

    cross_P[0][0] = vect_A[1][0] * vect_B[2][0] - vect_A[2][0] * vect_B[1][0];
    cross_P[1][0] = vect_A[0][0] * vect_B[2][0] - vect_A[2][0] * vect_B[0][0];
    cross_P[2][0] = vect_A[0][0] * vect_B[1][0] - vect_A[1][0] * vect_B[0][0];
}

void Rot_Matrix (double arr[3][3],double alpha, double theta)
{
    arr[0][0] = cos(theta);
    arr[0][1] = -sin(theta)*cos(alpha);
    arr[0][2] = sin(theta)*sin(alpha);

    arr[1][0] = sin(theta);
    arr[1][1] = cos(theta)*cos(alpha);
    arr[1][2] = -cos(theta)*sin(alpha);

    arr[2][0] = 0;
    arr[2][1] = sin(alpha);
    arr[2][2] = cos(alpha);

}

void b_vector (int axis, double b[3][1], double theta[7])
{
    int i,j,k;
    double bo[3][1];
    bo[0][0]=0;
    bo[1][0]=0;
    bo[2][0]=1;

    double R[7][3][3];
    double Rdum [3][3];
    /*Initialize Rotation matrix*/
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            if(i==j) Rdum[i][j]=1;
            else Rdum[i][j]=0;
        }
    }
    double Rres [3][3];

    double DH[7][4];
    DH_params(DH);
    for(i=0;i<7;i++)  DH[i][3]=theta[i];

    for(i=0;i<7;i++) /*Input rotation matrix*/
        Rot_Matrix(R[i],DH[i][0],DH[i][3]);

/*Initialize Rotation matrix*/
    if(axis>0)
    {
        for(i=0;i<3;i++)
            {
                for(j=0;j<3;j++)
                {
                    Rdum[i][j]=R[0][i][j];
                }
            }
    }

/*Multiply Consecutively*/
    for(i=0; i<(axis-1);i++)
    {
        multiplyG(3,3,Rdum,3,3,R[i+1],Rres);
        /*Passing value of the current result to A dummy*/
        for(j=0;j<3;j++)
        {
            for(k=0;k<3;k++)
            {
                /*printf("%f ", A_res[j][k]);*/
                Rdum[j][k]=Rres[j][k];

            }
        }
    }

    multiplyG(3,3,Rdum,3,1,bo,b);
}

void r_vector (int axis, double r[3][1], double theta[7])
{
    int i,j,k;
    double X[4][1], X1[4][1], X2[4][1];
    X[0][0] = 0;
    X[1][0] = 0;
    X[2][0] = 0;
    X[3][0] = 1;

    double DH[7][4];
    DH_params(DH);
    for(i=0;i<7;i++)  DH[i][3]=theta[i];

    double T[4][4], A[7][4][4], Adum[4][4], Ares[4][4];

    for(i = 0;i<7;i++)
    {
        Tranf_Matrix(A[i],DH[i][0],DH[i][1],DH[i][2],DH[i][3]);
    }

    if(axis>0)
    {
        for(i=0;i<4;i++)
            {
                for(j=0;j<4;j++)
                {
                    Adum[i][j]=A[0][i][j];
                }
            }
    }


    FK(theta,T);
    multiplyG(4,4,T,4,1,X,X1);

    for(i=0; i<(axis-1);i++)
    {
        multiply(Adum,A[i+1],Ares);
        /*Passing value of the current result to A dummy*/
        for(j=0;j<4;j++)
        {
            for(k=0;k<4;k++)
            {
                /*printf("%f ", A_res[j][k]);*/
                Adum[j][k]=Ares[j][k];

            }
        }
    }

    multiplyG(4,4,Adum,4,1,X,X2);

    if(axis==0)
    {
    r[0][0] = X1[0][0];
    r[1][0] = X1[1][0];
    r[2][0] = X1[2][0];
    }
    else
    {
        r[0][0] = X1[0][0] - X2[0][0];
        r[1][0] = X1[1][0] - X2[1][0];
        r[2][0] = X1[2][0] - X2[2][0];
    }

}

void Jacobian (double J[3][7], double theta[7])
{
    int i,j,k;
    double b[7][3][1], r[7][3][1], jprod[7][3][1];
    for(i=0;i<7;i++)
    {
        b_vector(i,b[i],theta);
        r_vector(i,r[i],theta);
        crossProduct(b[i],r[i],jprod[i]);
    }

    for(j=0;j<7;j++)
    {
        for(k=0;k<3;k++)
        {
            J[k][j] = jprod[j][k][1];
        }
    }

}

void Jac_calc (double J[3][7], double theta[7])
{
    double a,b,c,d,e,f,g;
    a = theta[0];
    b = theta[1];
    c = theta[2];
    d = theta[3];
    e = theta[4];
    f = theta[5];
    g = theta[6];

    J[0][0] = (-65*((-sin(a)*cos(b)*cos(c) - sin(c)*cos(a))*cos(d) + sin(a)*sin(b)*sin(d))*cos(e) - 65*(sin(a)*sin(c)*cos(b) - cos(a)*cos(c))*sin(e))*sin(f) + (65*(sin(a)*cos(b)*cos(c) + sin(c)*cos(a))*sin(d) + 65*sin(a)*sin(b)*cos(d))*cos(f) - (-350*sin(a)*cos(b)*cos(c) - 350*sin(c)*cos(a))*sin(d) + 350*sin(a)*sin(b)*cos(d) + 400*sin(a)*sin(b) - 100*sin(a);
    J[0][1] = (-65*(-sin(b)*cos(a)*cos(c)*cos(d) - sin(d)*cos(a)*cos(b))*cos(e) - 65*sin(b)*sin(c)*sin(e)*cos(a))*sin(f) + (65*sin(b)*sin(d)*cos(a)*cos(c) - 65*cos(a)*cos(b)*cos(d))*cos(f) + 350*sin(b)*sin(d)*cos(a)*cos(c) - 350*cos(a)*cos(b)*cos(d) - 400*cos(a)*cos(b);
    J[0][2] = (-65*(sin(a)*sin(c) - cos(a)*cos(b)*cos(c))*sin(e) - 65*(-sin(a)*cos(c) - sin(c)*cos(a)*cos(b))*cos(d)*cos(e))*sin(f) - (-350*sin(a)*cos(c) - 350*sin(c)*cos(a)*cos(b))*sin(d) + 65*(sin(a)*cos(c) + sin(c)*cos(a)*cos(b))*sin(d)*cos(f);
    J[0][3] = -65*(-(-sin(a)*sin(c) + cos(a)*cos(b)*cos(c))*sin(d) - sin(b)*cos(a)*cos(d))*sin(f)*cos(e) + ((65*sin(a)*sin(c) - 65*cos(a)*cos(b)*cos(c))*cos(d) + 65*sin(b)*sin(d)*cos(a))*cos(f) + (350*sin(a)*sin(c) - 350*cos(a)*cos(b)*cos(c))*cos(d) + 350*sin(b)*sin(d)*cos(a);
    J[0][4] = (-(-65*(-sin(a)*sin(c) + cos(a)*cos(b)*cos(c))*cos(d) + 65*sin(b)*sin(d)*cos(a))*sin(e) + (65*sin(a)*cos(c) + 65*sin(c)*cos(a)*cos(b))*cos(e))*sin(f);
    J[0][5] = (-65*((-sin(a)*sin(c) + cos(a)*cos(b)*cos(c))*cos(d) - sin(b)*sin(d)*cos(a))*cos(e) - 65*(-sin(a)*cos(c) - sin(c)*cos(a)*cos(b))*sin(e))*cos(f) - (65*(sin(a)*sin(c) - cos(a)*cos(b)*cos(c))*sin(d) - 65*sin(b)*cos(a)*cos(d))*sin(f);
    J[0][6] = 0;

    J[1][0] = (-65*((-sin(a)*sin(c) + cos(a)*cos(b)*cos(c))*cos(d) - sin(b)*sin(d)*cos(a))*cos(e) - 65*(-sin(a)*cos(c) - sin(c)*cos(a)*cos(b))*sin(e))*sin(f) + (65*(sin(a)*sin(c) - cos(a)*cos(b)*cos(c))*sin(d) - 65*sin(b)*cos(a)*cos(d))*cos(f) - (-350*sin(a)*sin(c) + 350*cos(a)*cos(b)*cos(c))*sin(d) - 350*sin(b)*cos(a)*cos(d) - 400*sin(b)*cos(a) + 100*cos(a);
    J[1][1] = (-65*(-sin(a)*sin(b)*cos(c)*cos(d) - sin(a)*sin(d)*cos(b))*cos(e) - 65*sin(a)*sin(b)*sin(c)*sin(e))*sin(f) + (65*sin(a)*sin(b)*sin(d)*cos(c) - 65*sin(a)*cos(b)*cos(d))*cos(f) + 350*sin(a)*sin(b)*sin(d)*cos(c) - 350*sin(a)*cos(b)*cos(d) - 400*sin(a)*cos(b);
    J[1][2] = (-65*(-sin(a)*sin(c)*cos(b) + cos(a)*cos(c))*cos(d)*cos(e) - 65*(-sin(a)*cos(b)*cos(c) - sin(c)*cos(a))*sin(e))*sin(f) - (-350*sin(a)*sin(c)*cos(b) + 350*cos(a)*cos(c))*sin(d) + 65*(sin(a)*sin(c)*cos(b) - cos(a)*cos(c))*sin(d)*cos(f);
    J[1][3] = ((-65*sin(a)*cos(b)*cos(c) - 65*sin(c)*cos(a))*cos(d) + 65*sin(a)*sin(b)*sin(d))*cos(f) - 65*(-(sin(a)*cos(b)*cos(c) + sin(c)*cos(a))*sin(d) - sin(a)*sin(b)*cos(d))*sin(f)*cos(e) + (-350*sin(a)*cos(b)*cos(c) - 350*sin(c)*cos(a))*cos(d) + 350*sin(a)*sin(b)*sin(d);
    J[1][4] = (-(-65*(sin(a)*cos(b)*cos(c) + sin(c)*cos(a))*cos(d) + 65*sin(a)*sin(b)*sin(d))*sin(e) + (65*sin(a)*sin(c)*cos(b) - 65*cos(a)*cos(c))*cos(e))*sin(f);
    J[1][5] = (-65*((sin(a)*cos(b)*cos(c) + sin(c)*cos(a))*cos(d) - sin(a)*sin(b)*sin(d))*cos(e) - 65*(-sin(a)*sin(c)*cos(b) + cos(a)*cos(c))*sin(e))*cos(f) - (65*(-sin(a)*cos(b)*cos(c) - sin(c)*cos(a))*sin(d) - 65*sin(a)*sin(b)*cos(d))*sin(f);
    J[1][6] = 0;

    J[2][0] = 0;
    J[2][1] = (-65*(-sin(b)*sin(d) + cos(b)*cos(c)*cos(d))*cos(e) + 65*sin(c)*sin(e)*cos(b))*sin(f) + (-65*sin(b)*cos(d) - 65*sin(d)*cos(b)*cos(c))*cos(f) - 350*sin(b)*cos(d) - 400*sin(b) - 350*sin(d)*cos(b)*cos(c);
    J[2][2] = (65*sin(b)*sin(c)*cos(d)*cos(e) + 65*sin(b)*sin(e)*cos(c))*sin(f) + 65*sin(b)*sin(c)*sin(d)*cos(f) + 350*sin(b)*sin(c)*sin(d);
    J[2][3] =  -65*(-sin(b)*sin(d)*cos(c) + cos(b)*cos(d))*sin(f)*cos(e) + (-65*sin(b)*cos(c)*cos(d) - 65*sin(d)*cos(b))*cos(f) - 350*sin(b)*cos(c)*cos(d) - 350*sin(d)*cos(b);
    J[2][4] = (-(-65*sin(b)*cos(c)*cos(d) - 65*sin(d)*cos(b))*sin(e) + 65*sin(b)*sin(c)*cos(e))*sin(f);
    J[2][5] = (-65*(sin(b)*cos(c)*cos(d) + sin(d)*cos(b))*cos(e) + 65*sin(b)*sin(c)*sin(e))*cos(f) - (-65*sin(b)*sin(d)*cos(c) + 65*cos(b)*cos(d))*sin(f);
    J[2][6] = 0;

}
void PseudoJacobian (double J[3][7], double Jpseudo[7][3])
{
    double Jt[7][3], Jinv[3][3], Jdum[3][3];
    Jac_Trans(J,Jt);
    multiplyG(3,7,J,7,3,Jt,Jdum);
    Inv3x3(Jdum,Jinv);
    multiplyG(7,3,Jt,3,3,Jinv,Jpseudo);
}

void ResolvedMotionRate(double jointvel[7], double carvel[3], double theta[7])
{
    int i;
    double J[3][7], Jpseudo[7][3], jv[7][1], cv[3][1];
    for(i=0;i<3;i++) cv[i][1] = carvel[i];
    Jac_calc(J,theta);
    PseudoJacobian(J,Jpseudo);
    multiplyG(7,3,Jpseudo,3,1,cv,jv);
    for(i=0;i<7;i++) jointvel[i] = jv[i][1];
}

int main()
{
    int i,j;
    double J[3][7], theta[7], Jps[7][3];
     for(int i=0; i<7;i++)
    {
        printf("Enter the value of Theta_%d:", i+1);
        scanf("%lf", &theta[i]);
        theta[i]=theta[i]*val;
    }

    Jac_calc(J, theta);
    PseudoJacobian(J, Jps);

    for(i=0;i<3;i++)
    {
        for(j=0;j<7;j++)
        {
            printf("%lf ", J[i][j]);
        }
        printf("\n");
    }
    printf("\n");

     /*for(i=0;i<7;i++)
    {
        for(j=0;j<3;j++)
        {
            printf("%lf ", Jps[i][j]);
        }
        printf("\n");
    }
    printf("\n");*/

     return 0;
}

