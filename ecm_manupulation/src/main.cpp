//#include "ecm_manupulation/Dynamics.h"
#include "ecm_manupulation/Kinematics.h"


int main(int argc, char* argv[])
{


    Kinematics kinematics;
    Matrix4f T_4_0_ = kinematics.computeFK(std::vector<float>{-0.3, 0.2, 0.1, -0.9,});

    std::cout << "T_4_0_: " << std::endl << T_4_0_ << std::endl;

//    kinematics.testIK(std::vector<float>{-0.3, 0.2, 0.1, -0.9,});
//    kinematics.testParams();


    return 0;
}
