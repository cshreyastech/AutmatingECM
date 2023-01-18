#ifndef _DYNAMICS_
#define _DYNAMICS_

#include "PCH/pch.h"
#include "JointKinematics.h"

class Dynamics
{
public:
  Dynamics();
    void InverseDynamics(const Vector4f qs, const Vector4f qds, const Vector4f qdds);
  ~Dynamics();

private:
  Vector4f tau_;
  // yawlink, pitchendlink, maininsertionlink, toollink
  const float m1 = 6.417f;
  const float m2 = 2.032f;
  const float m3 = 0.23f;
  const float m4 = 1.907f;

  const float Izz1 = 0.03268546890880477f;
  const float Izz2 = 0.05958032620936474f;
  const float Izz3 = 0.00030069653619773637f;
  const float Izz4 = 0.0018663962366877794f;

  const float L_rcc = 0.3822; const float L_scopelen = 0.385495;
  const float g = 9.81f;
};

#endif