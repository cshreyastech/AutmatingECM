#ifndef EigenUtilities_H
#define EigenUtilities_H

#include<iostream>
#include<Eigen/Dense>
#include<Eigen/Eigen>
#include<Eigen/Core>
#include<cmath>
#include <stdlib.h>

using namespace Eigen;
class EigenUtilities
{
public:
    EigenUtilities();
    float get_angle(Vector3f vec_a, Vector3f vec_b, Vector3f up_vector);
    float get_random_between_range(float low, float high);
    Eigen::Matrix3f rotation_from_euler(float roll, float pitch, float yaw);
    Eigen::Vector3f rpy_from_rotation(Eigen::Matrix3f R);
    Eigen::Matrix4f get_frame(Eigen::Matrix3f R, Vector3f T);
    ~EigenUtilities(void);
};

#endif // EigenUtilities_H
