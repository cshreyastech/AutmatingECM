#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include "PCH/pch.h"

using namespace Eigen;
class Utilities
{
public:
    Utilities();
    float GetAngle(Vector3f vec_a, Vector3f vec_b, Vector3f up_vector);
    float GetRandomBetweenRange(float low, float high);
    Matrix3f RotationFromEuler(Vector3f R_rpy);
    Vector3f RPYfromRotation(Matrix3f R);
    Matrix4f GetFrame(Matrix3f R, Vector3f T);
    ~Utilities(void);
};

#endif // UTILITIES_H