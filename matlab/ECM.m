

function ECM
    %% Clean up
    
    close all
    clc            % clear the command window
    clear all      % clear all variables
    format compact % condense command window output
    format long g  % +, bank, hex, long, rat, short, short g, short eng
    
    %% Symbolicy DH parameters of the joints
    m_baselink = 0.0; m_yawlink = 6.417; m_pitchbacklink = 0.421;
    m_pitchbottomlink = 0.359; m_pitchendlink = 2.032; 
    m_maininsertionlink = 0.231; m_toollink = 1.907; 
    m_pitchfrontlink = 1.607; m_pitchtoplink = 0.439;
    
    I_baselink = [
                        0,                     0,                     0;
                        0,                     0,                     0;
                        0,                     0,                     0;
        ];
    I_yawlink  = [
       0.0544029283547049,                     0,                     0;
                        0,   0.05561704829744072,                     0;
                        0,                     0,   0.03268546890880477;
        ];
    I_pitchbacklink = [
     0.003379498955485313,                     0,                     0;
                        0, 0.0005841630549272367,                     0;
                        0,                      0, 0.003891678623363246;
        ];
    I_pitchbottomlink = [
     0.0002523977891357399,                     0,                     0;
                        0,   0.005115360826245603,                     0;
                        0,                      0,  0.005207910357124296;
        ];
    I_pitchendlink = [
     0.05716144218253441,                     0,                      0;
                        0, 0.0033692502560575804,                     0;
                        0,                      0,  0.05958032620936474;
        ];
    I_maininsertionlink = [
     0.00008394349057701179,                     0,                      0;
                        0,  0.00033748904053492357,                     0;
                        0,                      0,  0.00030069653619773637;
        ];
    
    I_pitchfrontlink = [
     0.019475797762542296,                     0,                      0;
                        0,  0.010125186032029653,                      0;
                        0,                      0,  0.026757303110765244;
        ];
    I_pitchtoplink = [
    0.000021883602134626157,                  0,                      0;
                        0, 0.004746163141812917,                      0;
                        0,                    0,   0.004753741066639586;
        ];
    
    I_toollink =  [
     0.05913949950630815,                     0,                     0;
                        0,  0.05892034903069629,                     0;
                        0,                      0, 0.0018663962366877794;
        ];
    
    syms q_baselink_yawlink q_yawlink_pitchbacklink ... 
        q_pitchbacklink_pitchbottomlink q_pitchbottomlink_pitchendlink ...
        q_pitchendlink_maininsertionlink q_maininsertionlink_toollink ...
        q_yawlink_pitchfrontlink q_pitchfrontlink_pitchbottomlink ...
        q_pitchfrontlink_pitchtoplink q_pitchtoplink_pitchtoplink;
    
    syms qd_baselink_yawlink qd_yawlink_pitchbacklink ... 
        qd_pitchbacklink_pitchbottomlink qd_pitchbottomlink_pitchendlink ...
        qd_pitchendlink_maininsertionlink qd_maininsertionlink_toollink ...
        qd_yawlink_pitchfrontlink qd_pitchfrontlink_pitchbottomlink ...
        qd_pitchfrontlink_pitchtoplink qd_pitchtoplink_pitchtoplink;
    
    L_rcc = 0.3822;
    L_scopelen = 0.385495;
    round_decimal=3;
    %% Forward Kinematics
    % digits(4)
    % sympref('FloatingPointOutput',true);
    % T_baselink_yawlink = simplify(DH_Modified(pi/2.0, 0, q_baselink_yawlink, 0, pi/2.0, 'R'))
    
    T_baselink_yawlink = ... 
        vpa(simplify(DH_Modified(pi/2.0, 0, q_baselink_yawlink, 0, pi/2.0, 'R', round_decimal)), round_decimal);
    T_yawlink_pitchbacklink = ...
        vpa(simplify(DH_Modified(-pi/2.0, 0, q_yawlink_pitchbacklink, 0, -pi/2.0, 'R', round_decimal)), round_decimal);
    T_pitchendlink_maininsertionlink = ...
        vpa(simplify(DH_Modified(pi/2.0, 0, 0, q_pitchendlink_maininsertionlink, -L_rcc, 'P', round_decimal)), round_decimal);
    T_maininsertionlink_toollink = ...
        vpa(simplify(DH_Modified(0, 0, q_maininsertionlink_toollink, L_scopelen, 0, 'R', round_decimal)), round_decimal);
    
    T_W_baselink = [
        1, 0, 0,    0.5;
        0, 1, 0, 0.3901;
        0, 0, 1,   -0.6;
        ];
    
    T_W_yawlink           = vpa(simplify(T_W_baselink          * T_baselink_yawlink), round_decimal);
    T_W_pitchbacklink     = vpa(simplify(T_W_yawlink           * T_yawlink_pitchbacklink), round_decimal);
    T_W_maininsertionlink = vpa(simplify(T_W_pitchbacklink     * T_pitchendlink_maininsertionlink), round_decimal);
    T_W_toollink          = vpa(simplify(T_W_maininsertionlink * T_maininsertionlink_toollink), round_decimal);
    
    %% Jacobian
    J_W_v_yawlink(1:3, 1) = diff(T_W_yawlink(1:3, 4), q_baselink_yawlink);
    J_W_v_yawlink(1:3, 2) = zeros;
    J_W_v_yawlink(1:3, 3) = zeros;
    J_W_v_yawlink(1:3, 4) = zeros;
    
    J_W_w_yawlink(1:3, 1) = zeros;
    J_W_w_yawlink(  3, 1) = 1;
    J_W_w_yawlink(1:3, 2) = zeros;
    J_W_w_yawlink(1:3, 3) = zeros;
    J_W_w_yawlink(1:3, 4) = zeros;
    % --
    
    J_W_v_pitchbacklink(1:3, 1) = diff(T_W_pitchbacklink(1:3, 4), q_baselink_yawlink);
    J_W_v_pitchbacklink(1:3, 2) = diff(T_W_pitchbacklink(1:3, 4), q_yawlink_pitchbacklink);
    J_W_v_pitchbacklink(1:3, 3) = zeros;
    J_W_v_pitchbacklink(1:3, 4) = zeros;
    J_W_w_pitchbacklink = J_W_w_yawlink;
    J_W_w_pitchbacklink(  3, 2) = 1;
    % --
    
    J_W_v_maininsertionlink(1:3, 1) = diff(T_W_maininsertionlink(1:3, 4), q_baselink_yawlink);
    J_W_v_maininsertionlink(1:3, 2) = diff(T_W_maininsertionlink(1:3, 4), q_yawlink_pitchbacklink);
    J_W_v_maininsertionlink(1:3, 3) = diff(T_W_maininsertionlink(1:3, 4), q_pitchendlink_maininsertionlink);
    J_W_v_maininsertionlink(1:3, 4) = zeros;
    J_W_w_maininsertionlink = J_W_w_pitchbacklink;
    % --
    
    J_W_v_toollink(1:3, 1) = diff(T_W_toollink(1:3, 4), q_baselink_yawlink);
    J_W_v_toollink(1:3, 2) = diff(T_W_toollink(1:3, 4), q_yawlink_pitchbacklink);
    J_W_v_toollink(1:3, 3) = diff(T_W_toollink(1:3, 4), q_pitchendlink_maininsertionlink);
    J_W_v_toollink(1:3, 4) = diff(T_W_toollink(1:3, 4), q_maininsertionlink_toollink);
    J_W_w_toollink = J_W_w_maininsertionlink;
    J_W_w_toollink(  3, 4) = 1;
    
    %% Mass Matrix Calculation
    J_W_v_yawlink_square           = m_yawlink           * simplify(J_W_v_yawlink'           * J_W_v_yawlink);
    J_W_v_pitchbacklink_square     = m_pitchbacklink     * simplify(J_W_v_pitchbacklink'     * J_W_v_pitchbacklink);
    J_W_v_maininsertionlink_square = m_maininsertionlink * simplify(J_W_v_maininsertionlink' * J_W_v_maininsertionlink);
    J_W_v_toollink_square          = m_toollink          * simplify(J_W_v_toollink'          * J_W_v_toollink);
    
    % All w values are constant. Verify Calcuations.
    J_W_w_yawlink_square           = J_W_w_yawlink'           * I_yawlink           * J_W_w_yawlink;
    J_W_w_pitchbacklink_square     = J_W_w_pitchbacklink'     * I_pitchbacklink     * J_W_w_pitchbacklink;
    J_W_w_maininsertionlink_square = J_W_w_maininsertionlink' * I_maininsertionlink * J_W_w_maininsertionlink;
    J_W_w_toollink_square          = J_W_w_toollink'          * I_toollink          * J_W_w_toollink;
    
    syms M_syms(n);
    M_syms(n) = simplify( ...
        J_W_v_yawlink_square + J_W_v_pitchbacklink_square + J_W_v_maininsertionlink_square + J_W_v_toollink_square + ...
        J_W_w_yawlink_square + J_W_w_pitchbacklink_square + J_W_w_maininsertionlink_square + J_W_w_toollink_square);
    
    % 4x4
    M = vpa(simplify(M_syms(n)), round_decimal);
    M = vpa(M, round_decimal);
%     simplify(M(1, 1))
%     simplify(M(2, 2))
%     simplify(M(3, 3))
%     simplify(M(4, 4))
% simplify(M(1, 2) == M(2, 1))
% simplify(M(1, 3) == M(3, 1))
% simplify(M(1, 4) == M(4, 1))
% simplify(M(2, 3) == M(3, 4))
% simplify(M(2, 4) == M(4, 2))
% simplify(M(3, 4) == M(4, 3))
%     %% Centrifugal and Coriolis Vector V
%     % v(q, qd) = Md*qd - 1/2* [ [qd' * (M)td1r * qd]; [qd' * (M)td2r * qd] ]
%     % v(q, qd) = C(q)*[qd^2] + B(q)*[qd * qd] )
%     % Used derivation from Christoffel Symbols
%     
%     dm11 = diff(M(1, 1), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(1, 1), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(1, 1), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(1, 1), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
% 
%     dm12 = diff(M(1, 2), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(1, 2), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(1, 2), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(1, 2), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
% 
%     dm13 = diff(M(1, 3), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(1, 3), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(1, 3), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(1, 3), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
% 
%     dm14 = diff(M(1, 4), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(1, 4), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(1, 4), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(1, 4), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
%     %---
%     dm21 = diff(M(2, 1), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(2, 1), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(2, 1), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(2, 1), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
% 
%     dm22 = diff(M(2, 2), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(2, 2), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(2, 2), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(2, 2), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
% 
%     dm23 = diff(M(2, 3), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(2, 3), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(2, 3), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(2, 3), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
% 
%     dm24 = diff(M(2, 4), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(2, 4), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(2, 4), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(2, 4), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
%     %---
%     dm31 = diff(M(3, 1), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(3, 1), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(3, 1), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(3, 1), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
% 
%     dm32 = diff(M(3, 2), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(3, 2), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(3, 2), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(3, 2), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
% 
%     dm33 = diff(M(3, 3), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(3, 3), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(3, 3), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(3, 3), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
% 
%     dm34 = diff(M(3, 4), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(3, 4), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(3, 4), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(3, 4), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
%     %---
%     dm41 = diff(M(4, 1), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(4, 1), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(4, 1), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(4, 1), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
% 
%     dm42 = diff(M(4, 2), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(4, 2), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(4, 2), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(4, 2), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
% 
%     dm43 = diff(M(4, 3), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(4, 3), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(4, 3), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(4, 3), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
% 
%     dm44 = diff(M(4, 4), q_baselink_yawlink)               * qd_baselink_yawlink ...
%          + diff(M(4, 4), q_yawlink_pitchbacklink)          * qd_yawlink_pitchbacklink  ...
%          + diff(M(4, 4), q_pitchendlink_maininsertionlink) * qd_pitchendlink_maininsertionlink ...
%          + diff(M(4, 4), q_maininsertionlink_toollink)     * qd_maininsertionlink_toollink;
%     %---
%     dM = [ 
%         dm11, dm12, dm13, dm14;
%         dm21, dm22, dm23, dm24;
%         dm31, dm32, dm33, dm34;
%         dm41, dm42, dm43, dm44;
%        ];
% 
% 
%     dqs = [ 
%         qd_baselink_yawlink; 
%         qd_yawlink_pitchbacklink; 
%         qd_pitchendlink_maininsertionlink; 
%         qd_maininsertionlink_toollink; 
%         ];
%     dM_dq_baselink_yawlink               = simplify(dqs' * diff(M, q_baselink_yawlink)               * dqs);
%     dM_dq_yawlink_pitchbacklink          = simplify(dqs' * diff(M, q_yawlink_pitchbacklink)          * dqs);
%     dM_dq_pitchendlink_maininsertionlink = simplify(dqs' * diff(M, q_pitchendlink_maininsertionlink) * dqs);
%     dM_dq_maininsertionlink_toollink     = simplify(dqs' * diff(M, q_maininsertionlink_toollink)     * dqs);
%     dM_dqs = [
%         dM_dq_baselink_yawlink;
%         dM_dq_yawlink_pitchbacklink;
%         dM_dq_pitchendlink_maininsertionlink;
%         dM_dq_maininsertionlink_toollink;
%         ];
% 
%     V_syms(n) = simplify(dM * dqs - (1/2) * dM_dqs);
% 
%     V = V_syms(n);
%     V(1)
%     collect(V(1), [qd_baselink_yawlink, qd_maininsertionlink_toollink])
    
%     coeffs(collect(V(1), [qd_baselink_yawlink, qd_yawlink_pitchbacklink]), [qd_baselink_yawlink, qd_yawlink_pitchbacklink])
% coeffs(collect(V(1), [qd_baselink_yawlink, qd_maininsertionlink_toollink]), [qd_baselink_yawlink, qd_maininsertionlink_toollink])
%% Verify
% vpa(simplify(dqs' * (1/2 * dM - V) * dqs), round_decimal) == 0
% vpa(simplify(1/2 * dM - V), round_decimal)
%     %% Gravity
%     syms g;
%     G_W = simplify(...
%         - J_W_v_yawlink'           * [ 0; 0; -m_yawlink           * g; ] ...
%         - J_W_v_pitchbacklink'     * [ 0; 0; -m_pitchbacklink     * g; ] ...
%         - J_W_v_maininsertionlink' * [ 0; 0; -m_maininsertionlink * g; ] ...
%         - J_W_v_toollink'          * [ 0; 0; -m_toollink          * g; ] ...
%     );
% 
%     %% Torque
%     syms ddq_baselink_yawlink ddq_yawlink_pitchbacklink ...
%          ddq_pitchendlink_maininsertionlink ddq_maininsertionlink_toollink;
%     ddqs = [ 
%         ddq_baselink_yawlink; 
%         ddq_yawlink_pitchbacklink; 
%         ddq_pitchendlink_maininsertionlink; 
%         ddq_maininsertionlink_toollink; 
%         ];
% 
%     T = M * ddqs + V + G_W;
end
%%
function T_N_Nplus1 = DH_Modified(alpha, a, theta, d, offset, joint_type, round_decimal)
    if(joint_type == 'R')
        theta = theta + offset;
    elseif(joint_type == 'P')
        d = d + offset;
    else
        fprintf("Invalid Joint type");
    end

    ca = round(double(cos(alpha)), round_decimal);
    sa = round(double(sin(alpha)), round_decimal);

    ct = cos(theta);
    st = sin(theta);

    
    T_N_Nplus1 = [ 
        ct,      -st,        0,       a;
   st * ca,  ct * ca,      -sa, -d * sa;
   st * sa,  ct * sa,       ca,  d * ca;
         0,        0,        0,       1;
         ];
end
    