% Clear Variables 
clc
clear all
close all
%%
% Define Variables
syms l1                   ... Length
     h                    ... Cross-Section thickness
     m1 m2                  ... Mass
     t g ...
     th1 d2    ... Funciton of time
     dth1 dd2 ....
     ddth1 ddd2 ...
     T_0_C1_syms(n) T_0_C2_syms(n) ... 
     Ixx1 Iyy1 Izz1 ...
     Ixx2 Iyy2 Izz2 ...
     I1_C1_syms(n) I2_C2_syms(n) ...
     M_syms(n) C_syms(n) B_syms(n) V_syms(n);  ... Centrifugal, Coriolis, combined forces


%%
% Kinematics
T_0_C1_syms(n) = [
        cos(th1), -sin(th1), 0, l1*cos(th1);
        sin(th1),  cos(th1), 0, l1*sin(th1);
               0,         0, 1,           0;
               0,         0, 0,           1;
    ];

T_0_C2_syms(n) = [
        cos(th1), -sin(th1), 0, d2*cos(th1);
        sin(th1),  cos(th1), 0, d2*sin(th1);
               0,         0, 1,           0;
               0,         0, 0,           1;
    ];

T_0_C1 = T_0_C1_syms(n);
T_0_C2 = T_0_C2_syms(n);
% Jacobinan
J_0_v1(1:3, 1) = diff(T_0_C1(1:3, 4), th1);
J_0_v1(1:3, 2) = zeros;


J_0_v2(1:3, 1) = diff(T_0_C2(1:3, 4), th1);
J_0_v2(1:3, 2) = diff(T_0_C2(1:3, 4), d2);

J_C1_w1(1:3, 1) = zeros;
J_C1_w1(  3, 1) = 1;
J_C1_w1(1:3, 2) = zeros;

J_C2_w2 = J_C1_w1;


% Dynamics
% Define Intertia Tensor
I1_C1_syms(n) = [
    Ixx1,    0,    0;
       0, Iyy1,    0;
       0,    0, Izz1;
    ];

I2_C2_syms(n) = [
    Ixx2,    0,    0;
       0, Iyy2,    0;
       0,    0, Izz2;
    ];
I1_C1 = I1_C1_syms(n);
I2_C2 = I2_C2_syms(n);

% Mass matrix calculation
J_0_v1_square = m1 * simplify(transpose(J_0_v1) * J_0_v1);
J_0_v2_square = m2 * simplify(transpose(J_0_v2) * J_0_v2);

J_0_w1_square = simplify(transpose(J_C1_w1) * I1_C1 * J_C1_w1);
J_0_w2_square = simplify(transpose(J_C2_w2) * I2_C2 * J_C2_w2);

% M = m1 * Jv1' * Jv1 + m2 * Jv2' * Jv2 
% + Jw1' * I1_c1 * Jw1 + Jw2' * I2_c2* Jw2
M_syms(n) = simplify(J_0_v1_square + J_0_v2_square + J_0_w1_square + J_0_w2_square);
M = M_syms(n);
%%
% Deriving Centrifugal and Coriolis forces
% td1r partial differntiation w.r.t theta1
% td2r partial differntiation w.r.t theta2
% v(q, qd) = Md*qd - 1/2* [ [qd' * (M)td1r * qd]; [qd' * (M)td2r * qd] ]
% v(q, qd) = C(q)*[qd^2] + B(q)*[qd * qd] 

dm11 = simplify(diff(M(1, 1), th1) * dth1 ...
              + diff(M(1, 1), d2)  * dd2);
dm12 = simplify(diff(M(1, 2), th1) * dth1 ...
              + diff(M(1, 2), d2)  * dd2);
dm21 = simplify(diff(M(2, 1), th1) * dth1 ...
              + diff(M(2, 1), d2)  * dd2);
dm22 = simplify(diff(M(2, 2), th1) * dth1 ...
              + diff(M(2, 2), d2)  * dd2);
dM = [ dm11, dm12;
       dm21, dm22;];
dq = [
        dth1;
        dd2
     ];


dMdtheta1 = simplify(dq' * diff(M, th1) * dq);
dMdd2     = simplify(dq' * diff(M,  d2) * dq);
% syms noise(n)
% noise(n) = [
%     d2*dd2*m2 + 3* dth1*dd2;
%     -d2*m2*dth1 - 5*dth1*dd2;
%     ];
% V_syms(n) = simplify(dM * dq - (1/2) * [dMdtheta1; dMdd2] + noise);

V_syms(n) = simplify(dM * dq - (1/2) * [dMdtheta1; dMdd2]);
V = V_syms(n);
% collect(V, [dth1, dd2])
% coeffs(V(1),[dth1, dd2]) 
% coeffs(V(2),[dth1, dd2]) 


%%
% Gravity
G_0 = simplify(- J_0_v1' * [ 0; -m1 * g; 0] - J_0_v2' * [ 0; -m2 * g; 0]);

%%
% Torque calculation
% M(ddq) + c dq^2 + Bdq' + G = T
ddq = [
        ddth1;
        ddd2
     ];

T = M * ddq + V + G_0