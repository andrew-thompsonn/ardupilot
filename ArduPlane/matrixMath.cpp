#include "matrixMath.h"

void matrixMathFuncs::matrixMathTest(int num){
    float testMat1[num], testMat2[num];
    for(uint8_t i = 0;i<num;i++) {
        testMat1[i] = 6;
        testMat2[i] = 2;
    }
    
    float outMat[num];
    mat_mul(testMat1, testMat2, outMat, num);

    for (int i = 0; i < num; i++) {
        for (int j = 0; j < num; j++)
            printf("%.10f  ", outMat[i * num + j]);
        printf("\n");
    }

}