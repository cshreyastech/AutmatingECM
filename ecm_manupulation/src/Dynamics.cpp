#include "ecm_manupulation/Dynamics.h"

Dynamics::Dynamics() : Kinematics(){ }

Eigen::Matrix<float, 4, 4> Dynamics::getMassMatrix() {
    Eigen::Matrix<float, 4, 4> massMatrix;

    return massMatrix;
}

Dynamics::~Dynamics(void) {

}
