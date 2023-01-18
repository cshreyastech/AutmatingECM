#include "ECM/Dynamics.h"

Dynamics::Dynamics()
{

}

void Dynamics::InverseDynamics(const Vector4f qs, const Vector4f qds, const Vector4f qdds)
{
  const float q1 = qs(0); const float qd1 = qds(0); 
  const float q2 = qs(1); const float qd2 = qds(1); 
  const float q3 = qs(2); const float qd3 = qds(2); 
  const float q4 = qs(3); const float qd4 = qds(3); 

  Matrix4f M; M.setZero();
  M(0, 0) = Izz1 + Izz2 + Izz3 + Izz4 + m4*pow(cos(q2),2)*pow((L_scopelen - L_rcc + q3), 2) + 
    m3*pow(cos(q2),2)*pow((L_rcc - q3), 2);
  M(0, 2) = Izz2 + Izz3 + Izz4;
  M(0, 3) = Izz4;

  M(1, 0) = M(0, 1);
  M(1, 1) = Izz2 + Izz3 + Izz4 + m3*pow((L_rcc - q3),2) + m4*pow((L_scopelen - L_rcc + q3), 2);
  M(1, 3) = Izz4;

  M(2, 2) = m3 + m4;

  M(3, 0) = M(0, 3);
  M(3, 1) = M(1, 3);
  M(3, 3) = Izz4;

  Matrix4f C; C.setZero();
  const Vector4f qdCentrigugal = qds.array().square();
  C(1, 0) = (sin(2*q2)*(pow(L_rcc, 2)*m3 + pow(L_rcc,2)*m4 + pow(L_scopelen, 2)*m4 + 
    m3*pow(q3,2) + m4*pow(q3,2) - 2*L_rcc*L_scopelen*m4 - 2*L_rcc*m3*q3 - 
    2*L_rcc*m4*q3 + 2*L_scopelen*m4*q3))/2;
  C(2, 0) = -pow(cos(q2), 2)*(L_scopelen*m4 - L_rcc*m4 - L_rcc*m3 + m3*q3 + m4*q3);
  C(2, 1) = L_rcc*m3 + L_rcc*m4 - L_scopelen*m4 - m3*q3 - m4*q3;

  MatrixXf B(4, 6); B.setZero();
  const VectorXf qdCoriolis = (VectorXf(6) << 
    qd1 * qd2,
    qd1 * qd3,
    qd1 * qd4,
    qd2 * qd3,
    qd2 * qd4,
    qd3 * qd4).finished();

  B(0, 0) = -sin(2*q2)*(pow(L_rcc,2)*m3 + pow(L_rcc,2)*m4 + pow(L_scopelen,2)*m4 + 
    m3*pow(q3,2) + m4*pow(q3,2) - 2*L_rcc*L_scopelen*m4 - 2*L_rcc*m3*q3 - 
    2*L_rcc*m4*q3 + 2*L_scopelen*m4*q3); 
  B(0, 1) = 2*pow(cos(q2), 2)*(L_scopelen*m4 - L_rcc*m4 - L_rcc*m3 + m3*q3 + m4*q3);
  B(1, 3) = 2*L_scopelen*m4 - 2*L_rcc*m4 - 2*L_rcc*m3 + 2*m3*q3 + 2*m4*q3;

  const Vector4f G_W = Vector4f(  
    g*m4*cos(q2)*sin(q1)*(L_scopelen - L_rcc + q3) - g*m3*cos(q2)*sin(q1)*(L_rcc - q3),
    g*m4*cos(q1)*sin(q2)*(L_scopelen - L_rcc + q3) - g*m3*cos(q1)*sin(q2)*(L_rcc - q3),
                                                          -g*cos(q1)*cos(q2)*(m3 + m4),
    0.0);
  
  const Vector4f Tau = M * qdds + C * qdCentrigugal + B * qdCoriolis;
  
}

Dynamics::~Dynamics()
{
  
}