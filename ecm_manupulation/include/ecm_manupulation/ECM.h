#ifndef ECM_H
#define ECM_H

#include<vector>
#include<cmath>
#include<iostream>

//#include "ambf_client/ambf_client.h"
#include "DH.h"
#include "Utilities.h"

enum class JointType{
    ROTATIONAL,
    PRISMATIC,
};


class ECM
{
public:
    ECM();
//    Matrix4f computeFK(std::vector<float> joint_pos);
//    std::vector<float> computeIK(Matrix4f T_4_0);
//    void testIK(const std::vector<float>);
//    Eigen::MatrixXf getJacobian(const std::vector<float> desired_q);

    void cleanup();
    ~ECM(void);

protected:
    const std::string joint_type_enum_to_str(JointType enumVal);

    std::vector<DH *> DH_Vector;
    const float L_rcc = 0.3822;
    const float L_scopelen = 0.385495;
//    Matrix4f T_1_0_;
//    Matrix4f T_2_0_;
//    Matrix4f T_3_0_;
//    Matrix4f T_4_0_;



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
