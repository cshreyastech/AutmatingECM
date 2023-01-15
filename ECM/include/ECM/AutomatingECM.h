#ifndef _AUTOMATINGECM_
#define _AUTOMATINGECM_

#include "ECM/AMBFWrapper.h"
#include "ECM/Kinematics.h"
#include "PCH/pch.h"

class AutomatingECM
{
public:
  AutomatingECM();
  ~AutomatingECM();

  void TestKinematics();
private:
  AMBFWrapper* ambfWrapper_{ nullptr };
};

#endif