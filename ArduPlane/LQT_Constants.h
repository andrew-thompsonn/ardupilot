// LQT Constants Library 
//
// F_LAT matrix
/*#define F_LAT float[6][6] = {{-0.2525,-0.1820,2.4656,-0.4519,0,0},{0.0125,0.0210,-0.2021,0.1292,0,0},{-0.0941,0.0562,-0.1471,0.6036,0,0},{0.0149,0.0451,0.1026,0.9979,0,0},{0.0616,0.0254,-0.0449,0.4636,1,0},{0.9463,0.1553,0.8813,4.5788,20.9919,1}};

//F_LON matrix
#define F_LON float[6][6] = {{0.8254,0.2697,-0.7972,-9.1877,0,0},{-0.0375,-0.0109,0.0457,0.04628,0,0},{0.0260,0.0060,-0.0159,-0.2312,0,0},{0.0244,-0.0294,0.0883,0.9036,0,0},{0.9334,0.1285,-0.3634,-4.7290,1,0},{-0.2704,0.6056,-1.6767,-20.1998,0,1}}

//G_LAT
#define G_LAT float[6][2] = {{-0.2541,0.2495},{-0.1143,-0.0156},{-0.0527,0.0137},{-0.1113,-0.0155},{-0.0100,-0.0156},{-0.1481,-0.0137}}

//G_LON matrax
#define G_LON float[6][2] = {{0.3124,5.5488},{-0.2572,-0.2877},{-0.0754,0.1450},{-0.0801,0.0629},{0.0937,2.8602},{0.6145,-0.5338}};*/

//Defining gain values
//circular
float GAINS_LAT_CIRCLE[2][6] = {
    {0.1338,0.2023,0.4519,4.4355,2.4290,0.0155},
    {0.0852,0.1220,0.2525,2.6521,1.5190,0.0100}
    };
float GAINS_LON_CIRCLE[2][6] = {
    {0.0043,-0.0151,0.0449,0.4706,-0.0097,-0.0016},
    {-0.0718,0.0068,-0.0195,-0.1674,-0.0023,0.0120}
    };

//small curves - semicircle of squircle
float GAINS_LAT_CURVE[2][6] = {
    {0.1338,0.2023,0.4519,4.4355,2.4290,0.0155},
    {0.0852,0.1220,0.2525,2.6521,1.5190,0.0100}
    };
float GAINS_LON_CURVE[2][6] = {
    {0.0043,-0.0151,0.0449,0.4706,-0.0097,-0.0016},
    {-0.0718,0.0068,-0.0195,-0.1674,-0.0023,0.0120}
    };

//straight line
float GAINS_LAT_LINE[2][6] = {
    {0.3722,0.3229,0.8161,7.3050,7.3625,0.0831},
    {0.2561,0.2052,0.5285,4.6685,5.1058,0.0607}
    };
float GAINS_LON_LINE[2][6] = {
    {0.0162,-0.0857,0.2525,2.6891,-0.0549,-0.0198},
    {-0.1516,0.0241,-0.0699,-0.6664,-0.0341,0.0293}
    };
