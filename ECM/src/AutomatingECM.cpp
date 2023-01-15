#include "ECM/AutomatingECM.h"

AutomatingECM::AutomatingECM()
{
  ambfWrapper_ = new AMBFWrapper();;
  // Control control;
}

void AutomatingECM::TestKinematics()
{
  Kinematics kinematics(ambfWrapper_);
  kinematics.TestIK();
}

AutomatingECM::~AutomatingECM()
{
  delete ambfWrapper_;
  std::cout << "~AutomatingECM()\n";
}



int main()
{ 
  AutomatingECM automatingECM;
  automatingECM.TestKinematics();
  
  return 0;
}