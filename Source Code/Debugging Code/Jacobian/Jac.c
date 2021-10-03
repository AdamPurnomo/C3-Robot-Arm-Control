#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "kinematics-Test.c"


void Inv3x3 (double a[3][3], double ainv[3][3])
{
    int i,j;
    double determinant;
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
