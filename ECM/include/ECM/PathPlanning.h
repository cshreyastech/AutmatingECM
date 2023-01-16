#ifndef _PATHPLANNING_
#define _PATHPLANNING_

#include "PCH/pch.h"
#include "ECM/JointKinematics.h"

class PathPlanning
{
public:
  PathPlanning();

  std::vector<JointKinematics> TestPathPlanning(
    const float t0, const float tf, const int npts,
    const float q0, const float v0, const float a0,
    const float qf, const float vf, const float af);
  ~PathPlanning();

private:

  
};

#endif