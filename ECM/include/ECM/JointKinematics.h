#ifndef _JOINTKINEMATICS_
#define _JOINTKINEMATICS_

#include "PCH/pch.h"
struct JointKinematics
{
  float q;
  float qd;
  float qdd;
  float t;
};

#endif