#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PI 3.14159265
#define val PI/180

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

void Tranf_Matrix(float arr[4][4],int alpha, int a, int d, int theta)
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

void DH_params(int arr[7][4])
{
    arr[0][0] = 90;
    arr[0][1] = 100;
    arr[0][2] = 320;
    printf("Enter the value of Theta_1:");
    scanf("%d", &arr[0][3]);

    arr[1][0] = -90;
    arr[1][1] = 0;
    arr[1][2] = 0;
    printf("Enter the value of Theta_2:");
    scanf("%d", &arr[1][3]);

    arr[2][0] = 90;
    arr[2][1] = 0;
    arr[2][2] = 400;
    arr[2][3] = 0;

    arr[3][0] = -90;
    arr[3][1] = 0;
    arr[3][2] = 0;
    printf("Enter the value of Theta_4:");
    scanf("%d", &arr[3][3]);

    arr[4][0] = 90;
    arr[4][1] = 0;
    arr[4][2] = 350;
    printf("Enter the value of Theta_5:");
    scanf("%d", &arr[4][3]);

    arr[5][0] = -90;
    arr[5][1] = 0;
    arr[5][2] = 0;
    printf("Enter the value of Theta_6:");
    scanf("%d", &arr[5][3]);

    arr[6][0] = 0;
    arr[6][1] = 0;
    arr[6][2] = 65;
    printf("Enter the value of Theta_7:");
    scanf("%d", &arr[6][3]);

    return 0;

}

int main()
{
    int DH[7][4];
    DH_params(DH); /*store value of DH Parameters*/

    float A[7][4][4];

    /*store value to each transformation matrix*/
    for(int i = 0;i<7;i++)
    {
        Tranf_Matrix(A[i],DH[i][0],DH[i][1],DH[i][2],DH[i][3]);
    }

    float A_res[4][4]; /*Result of matrix multiplication*/
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
                printf("%f ", A_res[j][k]);
                A_dummy[j][k]=A_res[j][k];

            }
            printf("\n");
        }
        printf("\n");
    }

    /*for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            printf("%f ", A[6][i][j]);
        }
        printf("\n");
    }*/

    return 0;
}
