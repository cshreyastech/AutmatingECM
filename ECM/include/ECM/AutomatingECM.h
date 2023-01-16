#ifndef _AUTOMATINGECM_
#define _AUTOMATINGECM_

#include "PCH/pch.h"
#include "ECM/AMBFWrapper.h"
#include "ECM/Kinematics.h"
#include "ECM/PathPlanning.h"
#include "ECM/Control.h"

class AutomatingECM
{
public:
  AutomatingECM();
  ~AutomatingECM();

  void TestKinematics();
  void TestDynamics();
  void TestPathPlanning();
  void TestControl();
private:
  AMBFWrapper* ambfWrapper_{ nullptr };
};

#endif