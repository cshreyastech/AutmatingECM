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
  //   0,          1,     2 * ts,     3 * ts^2,
  //   0,          1,     2 * te,     3 * te^2;

  // y << 0, 1, 0, 0;
  // std::cout << "A\n" << A << std::endl;
  // std::cout << "y\n" << y << std::endl;
  // x = A.inverse() * y;
  // std::cout << "x\n" << x << std::endl;

  // q1_t  = x(0) + 1 * x(1) * t + 1 * x(2) * pow(t, 2) + x(3) * pow(t, 3);
  // qd1_t = x(1) + 2 * x(2) * t + 3 * x(3) * pow(t, 2); 

  // std::cout << "t: " << t << ", q1_t: " << q1_t << ", qd1_t: " << qd1_t << std::endl;
}


std::vector<JointKinematics> PathPlanning::TestPathPlanning(
  const float t0, const float tf, const int npts,
  const float q0, const float v0, const float a0, 
  const float qf, const float vf, const float af)
{
  if(t0 > tf) throw ("Invalid time frame exception\n");

  MatrixXf A(6, 6);
  
  A << 
    1, t0, pow(t0, 2),     pow(t0, 3),      pow(t0, 4),      pow(t0, 5),
    0,  1,     2 * t0, 3 * pow(t0, 2), 4  * pow(t0, 3), 5  * pow(t0, 4),
    0,  0,          2,         6 * t0, 12 * pow(t0, 2), 20 * pow(t0, 3),
    1, tf, pow(tf, 2),     pow(tf, 3),      pow(tf, 4),      pow(tf, 5),
    0,  1,     2 * tf, 3 * pow(tf, 2),  4 * pow(tf, 3), 5  * pow(tf, 4),
    0,  0,          2,         6 * tf, 12 * pow(tf, 2), 20 * pow(tf, 3);

  VectorXf Y(6), X(6);
  Y << q0, v0, a0, qf, vf, af;

  X = A.inverse() * Y;
  
  // std::cout << "A: \n" << A << std::endl;
  // std::cout << "Y: \n" << Y << std::endl;
  // std::cout << "X: \n" << X << std::endl;
  float step = (tf - t0) / (npts - 1);
  // printf("t0: %f, tf: %f, npts: %d, step: %f: \n", t0, tf, npts, step);

  std::vector<JointKinematics> jointsKinematics;
  for(float t = t0; t <= tf; t+= step )
  {
    float q   = X(0) + X(1) * t +     X(2) * pow(t, 2) +     X(3) * pow(t, 3); 
    float qd  =        X(1)     + 2 * X(2) * t         + 3 * X(3) * pow(t, 2); 
    float qdd =                   2 * X(2)             + 6 * X(3) * t;

    JointKinematics jointKinematics;
    jointKinematics.q = q;
    jointKinematics.qd = qd;
    jointKinematics.qdd = qdd;
    jointsKinematics.emplace_back(jointKinematics);
    printf("t: %f, q: %f, qd: %f, qdd: %f\n", t, q, qd, qdd); 
  }

  return jointsKinematics;
}

PathPlanning::~PathPlanning()
{
  // std::cout << "~PathPlanning()\n";
}