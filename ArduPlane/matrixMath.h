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
        #define MAT_ALG_ACCURACY    1e-4f

        matrixMathFuncs();

        bool matrixMathTest(int num);

        void LQTMult(float gains[][6], float error[6], float ans[]);

};


