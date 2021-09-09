

function CS223A_HW5
%%

close all
clc            % clear the command window
clear all      % clear all variables
format compact % condense command window output
format long g  % +, bank, hex, long, rat, short, short g, short eng
%%
    % symbolicy DH parameters of the joints
    syms L1 L2 h                ...
         d2 m1 m2               ...
         theta1(t)  pris2(t)    ...
         t1r        p2r         ...
         td1r       pd2r        ...
         
        th1 = theta1;
        p2 = pris2;
         
   
    %%
        
    A1 = [
        [cos(th1), -1.0*sin(th1),   0, 0.5*L1*cos(th1)];
        [sin(th1), +1.0*cos(th1),   0, 0.5*L1*sin(th1)];
        [       0,             0,   1,               0];
        [       0,             0,   0,               1];
        ];

    A2 = [
        [cos(th1), -1.0*sin(th1),   0, 1.0*L1*cos(th1)];
        [sin(th1), +1.0*cos(th1),   0, 1.0*L1*sin(th1)];
        [       0,             0,   1,              p2];
        [       0,             0,   0,               1];
        ];
    
    x1 = 0.5 * L1 * cos(th1);
    y1 = 0.5 * L1 * sin(th1);
    z1 = 0.0;
    
    
    x2 = 1.0 * L1 * cos(th1);
    y2 = 1.0 * L1 * sin(th1);
    z2 = 1.0;
    
    %%
    % Jacobian Calculations
    
     J0v1 = [
           [diff(x1, t), 0];
           [diff(y1, t), 0];
           [diff(z1, t), 0];
          ];
         
     J0v2 = [
           [diff(x2, t), 0];
           [diff(y2, t), 0];
           [diff(z2, t), 1];
          ];

    J0w1 = [
           [0, 0];
           [0, 0];
           [1, 0];
          ];
    
    J0w2 = [
           [0, 0];
           [0, 0];
           [1, 0];
          ];

%      aP = { theta1, pris2, diff(theta1(t), t), diff(pris2(t), t) };
%      sP = {    t1r,   p2r,               td1r,              pd2r };

     aP = { diff(theta1(t), t), diff(pris2(t), t) };
     sP = {                  1,                 1 };

     J0v1 = subs(J0v1, aP, sP)
     J0v2 = subs(J0v2, aP, sP);
%%
% Calculate Inertia
    I1C1 = [
           [(m1/ 6) * h^2,                                    0,                          0];
           [            0,              (m1/ 12) * (L1^2 + h^2),                          0];
           [            0,                                    0,    (m1/ 12) * (L1^2 + h^2)];
          ];
      
    I2C2 = [
           [(m2/ 12) * (L2^2 + h^2),                         0,                           0];
           [                      0,   (m2/ 12) * (L2^2 + h^2),                           0];
           [                      0,                         0,            (m2/  6) * (h^2)];
          ];

      %%
      % Calculation of Mass Matrix
      MJv1 = m1 * simplify(J0v1.' * J0v1)
      MJv2 = m2 * simplify(J0v2.' * J0v2);
      
            size(MJv1)
      MJw1 = simplify(J0w1.' * I1C1 * J0w1);
      MJw2 = simplify(J0w2.' * I2C2 * J0w2);
      
      
      M = simplify(MJv1 + MJw1 + MJv2 + MJw2);
function A = compute_dh_matrix(a, alpha, d, theta)
    ca = cos(alpha);
    sa = sin(alpha);

    ct = cos(theta);
    st = sin(theta);

    
    A = [ ct     ,    -st    ,      0,          a;
          st * ca,    ct * ca,    -sa,    -d * sa;
          st * sa,    ct * sa,     ca,     d * ca;
          0      ,          0,      0,           1;];