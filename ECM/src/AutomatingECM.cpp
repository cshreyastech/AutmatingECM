#include "ECM/AutomatingECM.h"

AutomatingECM::AutomatingECM()
{
  // ambfWrapper_ = new AMBFWrapper();

}

void AutomatingECM::TestKinematics()
{
  Kinematics kinematics(ambfWrapper_);
  kinematics.TestIK();
}

void AutomatingECM::TestDynamics()
{
  // Kinematics kinematics(ambfWrapper_);
  // kinematics.TestIK();
}

void AutomatingECM::TestPathPlanning()
{
  PathPlanning pp;
  pp.TestPathPlanning(0, 1, 10, -45, 0, 540, 45, 0, -540);
}

void AutomatingECM::TestControl()
{
  // Kinematics kinematics(ambfWrapper_);
  // kinematics.TestIK();
}

AutomatingECM::~AutomatingECM()
{
  // delete ambfWrapper_;
  std::cout << "~AutomatingECM()\n";
}



int main()
{ 
  AutomatingECM automatingECM;
  // automatingECM.TestKinematics();
  automatingECM.TestPathPlanning();
  
  return 0;
}