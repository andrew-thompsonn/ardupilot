// Matrix math header file, add necesary libraries and function declerations here

/*Libraries:*/
//below header file contains already existing matrix math functions
//#include "../../Plane.h"
#include "AP_Math/AP_Math.h"
#include <stdio.h>

/*functions:*/

class matrixMathFuncs {

    public:
        typedef float Ftype;

        void matrixMathTest(int num);
        uint16_t get_random(void);
        void show_matrix(Ftype *A, int n);

};


