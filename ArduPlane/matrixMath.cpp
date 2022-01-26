#include "matrixMath.h"

void matrixMathFuncs::matrixMathTest(int num){
    float kValues[2][6] = {
        {0.1338, 0.2023, 0.4519, 4.4355, 2.4290, 0.0155},
        {0.0852, 0.1220, 0.2525, 2.6521, 1.5190, 0.0100}
    };
    float xError[1][6] = {
        {0,0,0,0,0,0}
    };
    /*for (int i = 0; i < 2; i++){
        for (int j = 0; j < 6; j++){
            printf("value in array at location %d %d: %f\n",i,j,kValues[i][j]);
        }
    }
    for (int i = 0; i < 6; i++){
        printf("error value is: %f\n",xError[0][i]);
    }*/

    LQTMult(kValues,xError);


}

uint16_t matrixMathFuncs::get_random(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 65535) + (m_z >> 16);
    m_w = 18000 * (m_w & 65535) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xF;
}


void matrixMathFuncs::show_matrix(Ftype *A, int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++)
            printf("%.10f  ", A[i * n + j]);
        printf("\n");
    }
}

bool matrixMathFuncs::compare_mat(const Ftype *A, const Ftype *B, const uint8_t n)
{
    for(uint8_t i = 0; i < n; i++) {
        for(uint8_t j = 0; j < n; j++) {
            if(fabsf(A[i*n + j] - B[i*n + j]) > MAT_ALG_ACCURACY) {
                return false;
            }
        }
    }
    return true;
}

float matrixMathFuncs::LQTMult(float gains[][6], float error[][6]) 
{
    for (int i = 0; i < 2; i++){
        for (int j = 0; j < 6; j++){
            printf("value in array at location %d %d: %f\n",i,j,gains[i][j]);
        }
    }
    for (int i = 0; i < 6; i++){
        printf("error value is: %f\n",error[0][i]);
    }

    return 0;
}

/*fast inverses
    Ftype test_mat[25],ident_mat[25];
    Ftype out_mat[25], out_mat2[25], mat[25];
    for(uint8_t i = 0;i<25;i++) {
        test_mat[i] = powf(-1,i)*get_random()/0.7f;
    }
    
    //Test for 5x5 matrix
    mat_identity(ident_mat, 5);
    show_matrix(ident_mat,5);
    if (mat_inverse(test_mat,mat,5) && mat_inverse(mat, out_mat2, 5)) {
        mat_mul(test_mat, mat, out_mat, 5);
    } else {
        printf("5x5 Matrix is Singular!\n");
        return;
    }
*/
