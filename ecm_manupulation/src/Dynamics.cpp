#include "ecm_manupulation/Dynamics.h"

Dynamics::Dynamics() /*: ECM()*/{ }

Eigen::Matrix<float, 4, 4> Dynamics::getMassMatrix() {
    Eigen::Matrix<float, 4, 4> massMatrix;

    return massMatrix;
}

Dynamics::~Dynamics(void) {

}
