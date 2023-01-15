#include "ECM/PathPlanning.h"

PathPlanning::PathPlanning()
{
  // double ts = 0, te = 1.0;
  // double a0 = 0, a1 = 0, a2 = 0, a3 = 0, a4 = 0;
  // Eigen::Matrix4d A;
  // Eigen::Vector4d y, x;

  // double q1_t, qd1_t, t = 1;
  // A << 
  //   1, pow(ts, 1), pow(ts, 2), pow(ts, 3),
  //   1, pow(te, 1), pow(te, 2), pow(te, 3),
  //   0,          1,     2 * ts,     3 * ts,
  //   0,          1,     2 * te,     3 * te;

  // y << 0, 1, 0, 0;
  // std::cout << "A\n" << A << std::endl;
  // std::cout << "y\n" << y << std::endl;
  // x = A.inverse() * y;
  // std::cout << "x\n" << x << std::endl;

  // q1_t  = x(0) + 1 * x(1) * t + 1 * x(2) * pow(t, 2) + x(3) * pow(t, 3);
  // qd1_t = x(1) + 2 * x(2) * t + 3 * x(3) * pow(t, 2); 

  // std::cout << "t: " << t << ", q1_t: " << q1_t << ", qd1_t: " << qd1_t << std::endl;
}

PathPlanning::~PathPlanning()
{
  // std::cout << "~PathPlanning()\n";
}