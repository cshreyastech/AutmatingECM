#ifndef ECM_H
#define ECM_H

#include<vector>
#include<cmath>
#include<iostream>

//#include "ambf_client/ambf_client.h"
#include "DH.h"
#include "Utilities.h"

class ECM
{
public:
    ECM();

    void cleanup();
    ~ECM(void);

protected:
    std::vector<DH *> DH_Vector;
    const float L_rcc = 0.3822;
    const float L_scopelen = 0.385495;


    float dh_params[4][6] = {
   //     alpha,      a,      theta,                d,                  offset,     joint_type
        { M_PI_2,     0.0,    0.0,                  0.0,                M_PI_2,     0},
        { -M_PI_2,    0.0,    0.0,                  0.0,                -M_PI_2,    0},
        { M_PI_2,     0.0,    0.0,                  0.0,                -L_rcc,    1},
        { 0.0,        0.0,    0.0,                  L_scopelen,            0.0,    0}
    };

    std::vector<std::vector<float>> ECM_JOINT_LIMITS = {
        {  (float) (-91.96 * (M_PI / 180.0)),  (float) (91.96 * (M_PI / 180.0))},
        {  (float) (-60.00 * (M_PI / 180.0)),  (float) (60.00 * (M_PI / 180.0))},
        {                    0.0,   0.24               },
        { (float) (-175.00 * (M_PI / 180.0)), (float) (175.00 * (M_PI / 180.0))}
    };


};

#endif // ECM_H
