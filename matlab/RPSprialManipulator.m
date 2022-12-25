% Clear Variables 
clc
clear all
close all
%%
% Define Variables
syms L1 L2                  ... Length
     h                    ... Cross-Section thickness
     m1 m2                  ... Mass
     theta1(t) d2(t)    ... Funciton of time
     t1r t2r                ... Replacement varailable for subs
     td1r td2r              ... Replacement varailable for subs
     t g                   ... Time and gravity
     T_0_C1_syms(n) T_0_C2_syms(n) ... Transformations
     I1_c1_syms(n) Ixx1 Iyy1 Izz1         ... Intertia Tensor1
     I2_c2_syms(n) Ixx2 Iyy2 Izz2        ... Intertia Tensor2
     C_syms(n) B_syms(n) V_syms(n);  ... Centrifugal, Coriolis, combined forces
th1 = theta1;
th2 = d2;
aP = {theta1,     d2, diff(theta1(t), t),  diff(d2(t), t)};
sP = {   t1r,    t2r,               td1r,            td2r};

%%
% Kinematics
T_0_C1_syms(n) = [
        cos(th1), -sin(th1), 0, 1/2*L1*cos(th1);
        sin(th1),  cos(th1), 0, 1/2*L1*sin(th1);
               0,         0, 1,               0;
               0,         0, 0,               1;
    ];

T_0_C2_syms(n) = [
        cos(th1), -sin(th1), 0,     L1*cos(th1);
        sin(th1),  cos(th1), 0,     L1*sin(th1);
               0,         0, 1,              d2;
               0,         0, 0,               1;
    ];

% Jacobinan


% 
% % https://www.mathworks.com/matlabcentral/answers/410630-how-to-call-element-of-matrix-of-symbolic-variables
T_0_C1 = T_0_C1_syms(n);
J_0_v1(1:3, 1) = subs(diff(subs(T_0_C1(1:3, 4), aP, sP), t1r), sP, aP);
J_0_v1(1:3, 2) = zeros;

T_0_C2 = T_0_C2_syms(n);
J_0_v2(1:3, 1) = subs(diff(subs(T_0_C2(1:3, 4), aP, sP), t1r), sP, aP);
J_0_v2(1:3, 2) = subs(diff(subs(T_0_C2(1:3, 4), aP, sP), t2r), sP, aP);

J_C1_w1(1:3, 1) = zeros;
J_C1_w1(  3, 1) = 1;
J_C1_w1(1:3, 2) = zeros;

J_C2_w2 = J_C1_w1;

%%
% Dynamics
% Define Intertia Tensor
I1_c1_syms(n) = [
    (m1/6) * h^2,                      0,                      0;
               0, (m1/12) * (L1^2 + h^2),                      0;
               0,                      0, (m1/12) * (L1^2 + h^2);
    ];
I1_c1 = I1_c1_syms(n);

I2_c2_syms(n) = [
    (m2/12) * (L2^2 + h^2),                      0,            0;
                         0, (m2/12) * (L2^2 + h^2),            0;
               0,                                0, (m2/6) * h^2;
    ];
I2_c2 = I2_c2_syms(n);

% Mass matrix calculation
J_0_v1_square = m1 * simplify(transpose(J_0_v1) * J_0_v1);
J_0_v2_square = m2 * simplify(transpose(J_0_v2) * J_0_v2);

J_0_w1_square = simplify(transpose(J_C1_w1) * I1_c1 * J_C1_w1);
J_0_w2_square = simplify(transpose(J_C2_w2) * I2_c2 * J_C2_w2);

%M = m1 * Jv1' * Jv1 + m2 * Jv2' * Jv2 
% + Jw1' * I1_c1 * Jw1 + Jw2' * I2_c2* Jw2
M = simplify(J_0_v1_square + J_0_v2_square + J_0_w1_square + J_0_w2_square);

%%
% Deriving Centrifugal and Coriolis forces
% td1r partial differntiation w.r.t theta1
% td2r partial differntiation w.r.t theta2
% v(q, qd) = Md*qd - 1/2* [ [qd' * (M)td1r * qd]; [qd' * (M)td2r * qd] ]
% v(q, qd) = C(q)*[qd^2] + B(q)*[qd * qd] 
% simplify(diff(subs(diff(subs(M(1, 1), aP, sP), t1r), sP, aP), t))

% syms M_syms(n) ... Mass matrix symbolic
% M11(t) M12(t) M21(t) M22(t);
% M_syms(n) = [ 
%     M11(t), M12(t);
%     M21(t), M22(t);
%     ];
% M = M_syms(n);

dm11 = subs(diff(subs(M(1, 1), aP, sP), t1r), sP, aP) * diff(theta1(t), t) ...
     + subs(diff(subs(M(1, 1), aP, sP), t2r), sP, aP) * diff(d2(t), t);
dm12 = subs(diff(subs(M(1, 2), aP, sP), t1r), sP, aP) * diff(theta1(t), t) ...
     + subs(diff(subs(M(1, 2), aP, sP), t2r), sP, aP) * diff(d2(t), t);
dm21 = subs(diff(subs(M(2, 1), aP, sP), t1r), sP, aP) * diff(theta1(t), t) ...
     + subs(diff(subs(M(2, 1), aP, sP), t2r), sP, aP) * diff(d2(t), t);
dm22 = subs(diff(subs(M(2, 2), aP, sP), t1r), sP, aP) * diff(theta1(t), t) ...
     + subs(diff(subs(M(2, 2), aP, sP), t2r), sP, aP) * diff(d2(t), t);
dM = [ dm11, dm12;
       dm21, dm22;];
dq = [
        diff(theta1(t), t);
        diff(d2(t), t)
     ];

M1 = dq' * [ 
    subs(diff(subs(M(1, 1), aP, sP), t1r), sP, aP), subs(diff(subs(M(1, 2), aP, sP), t1r), sP, aP);
    subs(diff(subs(M(2, 1), aP, sP), t1r), sP, aP), subs(diff(subs(M(2, 2), aP, sP), t1r), sP, aP);
    ] * dq;

M2 = dq' * [ 
    subs(diff(subs(M(1, 1), aP, sP), t2r), sP, aP), subs(diff(subs(M(1, 2), aP, sP), t2r), sP, aP);
    subs(diff(subs(M(2, 1), aP, sP), t2r), sP, aP), subs(diff(subs(M(2, 2), aP, sP), t2r), sP, aP);
    ] * dq;

V = dM * dq - (1/2) * [M1; M2];
%%
% Gravity
G_0 = - J_0_v1' * [ 0; 0; m1 * g] - J_0_v2' * [ 0; 0; m2 * g];

%%
% Torque calculation
% M(ddq) + c dq^2 + Bdq' + G = T
T = M * [diff(theta1(t), t, t); diff(d2(t), t, t)] + V + G_0