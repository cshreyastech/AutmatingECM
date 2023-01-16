#ifndef _CONTROL_
#define _CONTROL_

#include "PCH/pch.h"
#include "ECM/PathPlanning.h"

class Control
{
public:
  Control(const Matrix4f T_4_w);
  ~Control();

private:
  Matrix4f T_4_w_;
 
};

#endif