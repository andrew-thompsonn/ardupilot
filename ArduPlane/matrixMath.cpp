#include "matrixMath.h"

void matrixMathFuncs::matrixMathTest(int num){
    //Test for 3x3 matrix
    Ftype test_mat[25],ident_mat[25];
    Ftype out_mat[25], out_mat2[25], mat[25];
    for(uint8_t i = 0;i<25;i++) {
        test_mat[i] = powf(-1,i)*get_random()/0.7f;
    }


    //Test for 3x3 matrix
    mat_identity(ident_mat, 3);
    if (mat_inverse(test_mat,mat,3) && mat_inverse(mat, out_mat2, 3)) {
        mat_mul(test_mat, mat, out_mat, 3);
    } else {
        printf("3x3 Matrix is Singular!\n");
        return;

    }
    printf("\n\n3x3 Test Matrix:\n");
    show_matrix(test_mat,3);
    printf("\nInverse of Inverse of matrix\n");
    show_matrix(mat,3);
    printf("\nInv(A) * A\n");
    show_matrix(out_mat,3);
    printf("\n");
    
    /*float testMat1[num], testMat2[num];
    for(uint8_t i = 0;i<num;i++) {
        testMat1[i] = 6;
        testMat2[i] = 2;
	printf("testMat1 val: %f\n testMat2 val: %f\n",testMat1[i],testMat2[i]);
    }
    
    // error seems to be coming from the use of mat_mul
    float outMat[num];
    mat_mul(testMat1, testMat2, outMat, num);

    for (int i = 0; i < num; i++) {
        for (int j = 0; j < num; j++)
            printf("%.10f  ", outMat[i * num + j]);
        printf("\n");
    }
    */	

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