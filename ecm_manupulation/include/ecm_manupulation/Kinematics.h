#ifndef KINEMATICS_H
#define KINEMATICS_H
#include "ecm_manupulation/ECM.h"
using namespace ECMSpecs;

class Kinematics : public ECM
{
public:
    Kinematics();
    Matrix4f computeFK(std::vector<float> joint_pos);
    std::vector<float> computeIK(Matrix4f T_4_0);

    void testIK(const std::vector<float>);
    Eigen::MatrixXf getJacobian(const std::vector<float> desired_q);


    void testParams();
    void cleanup();
    ~Kinematics(void);

private:
    Matrix4f T_1_0_;
    Matrix4f T_2_0_;
    Matrix4f T_3_0_;
    Matrix4f T_4_0_;

};



#endif // KINEMATICS_H
