#include "ECM/Control.h"

Control::Control(Matrix4f T_4_w) : T_4_w_(T_4_w)
{
  // PathPlanning pp;
}

Control::~Control()
{
  std::cout << "~Control()\n";
}