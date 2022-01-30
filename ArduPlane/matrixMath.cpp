#include "matrixMath.h"

void matrixMathFuncs::matrixMathTest(int num){
    float ans[2];
    float kValues[2][6] = {
        {0.1338, 0.2023, 0.4519, 4.4355, 2.4290, 0.0155},
        {0.0852, 0.1220, 0.2525, 2.6521, 1.5190, 0.0100}
    };

    float xError[6] = {1,5,66,85,2,20};

    LQTMult(kValues,xError,ans);

    for (int i = 0; i < 2; i++) {
        printf("Computed control input in index %d is %f\n",i,ans[i]);
    }


}

void matrixMathFuncs::LQTMult(float gains[][6], float error[6], float ans[]) 
{
    /*for (int i = 0; i < 2; i++){
        for (int j = 0; j < 6; j++){
            printf("value in array at location %d %d: %f\n",i,j,gains[i][j]);
        }
    }
    for (int i = 0; i < 6; i++){
        printf("error value is: %f\n",error[i]);
    }*/

    float sum; 
    for (int i = 0; i<2; i++) {
        sum = 0;
        for (int j = 0; j<6; j++) {
            sum = sum + gains[i][j]*error[j];
        }
        ans[i] = sum;
    }

}
