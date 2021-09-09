

function ECM
%%

close all
clc            % clear the command window
clear all      % clear all variables
format compact % condense command window output
format long g  % +, bank, hex, long, rat, short, short g, short eng

%%
    % symbolicy DH parameters of the joints


    
     syms a1 alpha1 depth1    theta1(t)  th1r thD1r ...
         a2 alpha2 depth2    theta2(t)  th2r thD2r ...
         a3 alpha3 depth3(t) theta3     de3r deD3r ...
         a4 alpha4 depth4    theta4(t)  th4r thD4r ...
         t g ...
         L_r L_s;
     
    m1 = 1.0; m2 = 6.417; m3 = 0.421; m4 = 0.231;
    
%     L_r = 0.3822;
%     L_s = 0.385495;
    
    th1 = theta1;
    th2 = theta2;
    dep3 = depth3;
    
    th4 = theta4;

    %%
    
    % Substuting DH of joins and simplifying them
    
    compute_dh_matrix(0.0, pi / 2, 0.0, th1 + pi / 2);
    
    A1 = vpa(simplify(compute_dh_matrix(0.0, pi / 2, 0.0, th1 + pi / 2)));

    
    
%     % Translation matrix of joints with respect to base frame
    T01 = A1;
    T01 = simplify(T01);
    
%     T01 = vpa(T01, 2);
%     A = formula(T01);
%     A(2, 1)
%     calculate(T01)
%     T01 = subs(T01, [a1, alpha1, d1, theta1], [0, pi/2, 0, -0.3]);
%     vpa(T01,2)
    
    
    A2 = simplify(compute_dh_matrix(0.0, -pi / 2, 0.0, th2 - pi / 2));
    T02 = T01 ...
        * A2;
    T02 = simplify(T02);

    A3 = simplify(compute_dh_matrix(0.0, pi / 2, dep3 - L_r, 0.0));
    
    T03 = T02 ...
        * A3;

    A4 = simplify(compute_dh_matrix(0.0, 0.0, L_s, th4));
    T04 = T03 ...
        * A4;

    %%
    % Calculating Linear Velocity

%     

aP = {theta1, theta2, depth3, theta4, diff(theta1(t), t), diff(theta2(t), t), diff(depth3(t), t), diff(theta4(t), t) };
sP = {th1r, th2r, de3r, th4r, thD1r, thD2r, deD3r, thD4r};  


%sP{2}
    A = formula(T01);
    %J0v4=sym(zeros(3,4));
    %J0v1 = simplify(subs(diff(subs(A(1:3, 4), aP, sP), th1r), sP, aP))
    
         J0v1 = [
           [subs(diff(subs(A(1, 4), aP, sP), sP{1}), sP, aP), 0, 0, 0 ];
           [subs(diff(subs(A(2, 4), aP, sP), sP{1}), sP, aP), 0, 0, 0 ];
           [subs(diff(subs(A(3, 4), aP, sP), sP{1}), sP, aP), 0, 0, 0 ];
          ];
    
    % J0v2 = simplify(subs(diff(subs(A(1:3, 4), aP, sP), th2r), sP, aP));
    A = formula(T02);
      J0v2 = [
           [subs(diff(subs(A(1, 4), aP, sP), sP{1}), sP, aP), subs(diff(subs(A(1, 4), aP, sP), sP{2}), sP, aP), 0, 0 ];
           [subs(diff(subs(A(2, 4), aP, sP), sP{1}), sP, aP), subs(diff(subs(A(2, 4), aP, sP), sP{2}), sP, aP), 0, 0 ];
           [subs(diff(subs(A(3, 4), aP, sP), sP{1}), sP, aP), subs(diff(subs(A(3, 4), aP, sP), sP{2}), sP, aP), 0, 0 ];
          ];
    
    
    %J0v3 = simplify(subs(diff(subs(A(1:3, 4), aP, sP), de3r), sP, aP));
      A = formula(T03);
      J0v3 = [
           [subs(diff(subs(A(1, 4), aP, sP), sP{1}), sP, aP), subs(diff(subs(A(1, 4), aP, sP), sP{2}), sP, aP), subs(diff(subs(A(1, 4), aP, sP), sP{3}), sP, aP), 0 ];
           [subs(diff(subs(A(2, 4), aP, sP), sP{1}), sP, aP), subs(diff(subs(A(2, 4), aP, sP), sP{2}), sP, aP), subs(diff(subs(A(2, 4), aP, sP), sP{3}), sP, aP), 0 ];
           [subs(diff(subs(A(3, 4), aP, sP), sP{1}), sP, aP), subs(diff(subs(A(3, 4), aP, sP), sP{2}), sP, aP), subs(diff(subs(A(3, 4), aP, sP), sP{3}), sP, aP), 0 ];
          ];
      
    %J0v4 = simplify(subs(diff(subs(A(1:3, 4), aP, sP), th4r), sP, aP));
    A = formula(T04);
      J0v4 = [
           [subs(diff(subs(A(1, 4), aP, sP), sP{1}), sP, aP), subs(diff(subs(A(1, 4), aP, sP), sP{2}), sP, aP), subs(diff(subs(A(1, 4), aP, sP), sP{3}), sP, aP), subs(diff(subs(A(1, 4), aP, sP), sP{4}), sP, aP) ];
           [subs(diff(subs(A(2, 4), aP, sP), sP{1}), sP, aP), subs(diff(subs(A(2, 4), aP, sP), sP{2}), sP, aP), subs(diff(subs(A(2, 4), aP, sP), sP{3}), sP, aP), subs(diff(subs(A(2, 4), aP, sP), sP{4}), sP, aP) ];
           [subs(diff(subs(A(3, 4), aP, sP), sP{1}), sP, aP), subs(diff(subs(A(3, 4), aP, sP), sP{2}), sP, aP), subs(diff(subs(A(3, 4), aP, sP), sP{3}), sP, aP), subs(diff(subs(A(3, 4), aP, sP), sP{4}), sP, aP) ];
          ];

%%

% Calculating Angular Velocity
A = formula(T01);
    %J0w1 = zeros(3, 4);
    J0w1 = [
           [A(1,3), 0, 0, 0];
           [A(2,3), 0, 0, 0];
           [A(3,3), 0, 0, 0];
          ];
    J0w1(1:3, 1) = A(1:3, 3);

A = formula(T02);
    J0w2 = [
           [0, A(1,3), 0, 0];
           [0, A(2,3), 0, 0];
           [0, A(3,3), 0, 0];
          ];


    J0w3 = [
           [0, 0, 0, 0];
           [0, 0, 0, 0];
           [0, 0, 0, 0];
          ];


A = formula(T04);
    J0w4 = [
           [0, 0, 0, A(1,3)];
           [0, 0, 0, A(2,3)];
           [0, 0, 0, A(3,3)];
          ];
    
    %%
% Calculate Inertia
% Values to be replaced with actuals from ADF file
    I1C1 = eye(3);
    

    
    I2C2 = eye(3);
    I3C3 = eye(3);
    I4C4 = eye(3);
          %%
      % Calculation of Mass Matrix
      MJv1 = m1 * simplify(J0v1.' * J0v1);
      MJv2 = m2 * simplify(J0v2.' * J0v2);
      MJv3 = m3 * simplify(J0v3.' * J0v3);
      MJv4 = m4 * simplify(J0v4.' * J0v4);
       

        MJw1 = simplify(J0w1.' * I1C1 * J0w1);
        MJw2 = simplify(J0w2.' * I2C2 * J0w2);
        MJw3 = (J0w3.' * I3C3 * J0w3);
        MJw4 = simplify(J0w4.' * I4C4 * J0w4);
      
      
      M = simplify(MJv1 + MJv2 + MJv3 + MJv4 + MJw1 + MJw2 + MJw3 + MJw4);
end

function A = compute_dh_matrix(a, alpha, d, theta)
    ca = cos(alpha);
    if(abs(ca) < 1e-6)
        ca = 0;
    end
    sa = sin(alpha);

    ct = cos(theta);
    st = sin(theta);

    
    A = [ ct     ,    -st    ,      0,          a;
          st * ca,    ct * ca,    -sa,    -d * sa;
          st * sa,    ct * sa,     ca,     d * ca;
          0      ,          0,      0,           1;];
end
    
function A = compute_linear_vel_matrix(a, alpha, d, theta)
    ca = cos(alpha);
    if(abs(ca) < 1e-6)
        ca = 0;
    end
    sa = sin(alpha);

    ct = cos(theta);
    st = sin(theta);

    
    A = [ ct     ,    -st    ,      0,          a;
          st * ca,    ct * ca,    -sa,    -d * sa;
          st * sa,    ct * sa,     ca,     d * ca;
          0      ,          0,      0,           1;];
end
    