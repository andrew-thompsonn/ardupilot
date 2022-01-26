#include "matrixMath.h"

void matrixMathFuncs::matrixMathTest(int num){
    

    //fast inverses
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