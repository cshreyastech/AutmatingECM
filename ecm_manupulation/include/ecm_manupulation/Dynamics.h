#ifndef DYNAMICS_H
#define DYNAMICS_H

#include"ecm_manupulation/ECM.h"
#include"ecm_manupulation/Kinematics.h"
#include"ecm_manupulation/Utilities.h"

class Dynamics/* : public ECM*/
{
public:
    Dynamics();

    Eigen::Matrix<float, 4, 4> getMassMatrix();

    ~Dynamics(void);
};

#endif // DYNAMICS_H
