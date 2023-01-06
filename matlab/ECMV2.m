

function ECM
    %% Clean up
    
    close all
    clc            % clear the command window
    clear all      % clear all variables
    format compact % condense command window output
    format long g  % +, bank, hex, long, rat, short, short g, short eng
    
    %% Symbolicy DH parameters of the joints
    syms m1 m2 m3 m4
%     m_baselink = 0.0; m_yawlink = 6.417; m_pitchbacklink = 0.421;
%     m_pitchbottomlink = 0.359; m_pitchendlink = 2.032; 
%     m_maininsertionlink = 0.231; m_toollink = 1.907; 
%     m_pitchfrontlink = 1.607; m_pitchtoplink = 0.439;
    
    syms Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 ...
         Ixx3 Iyy3 Izz3 Ixx4 Iyy4 Izz4;
    
    I1  = [
        Ixx1,    0,    0;
           0, Iyy1,    0;
           0,    0, Izz1;
        ];
    I2  = [
        Ixx2,    0,    0;
           0, Iyy2,    0;
           0,    0, Izz2;
        ];
    I3  = [
        Ixx3,    0,    0;
           0, Iyy3,    0;
           0,    0, Izz3;
        ];
    I4  = [
        Ixx4,    0,    0;
           0, Iyy4,    0;
           0,    0, Izz4;
        ];

    syms q1 q2 q3 q4 ...
         qd1 qd2 qd3 qd4 ...
         qdd1 qdd2 qdd3 qdd4;

    syms L_rcc L_scopelen;

    round_decimal=3;
    %% Forward Kinematics    
    T_baselink_yawlink = ... 
        simplify(DH_Modified(pi/2.0, 0, q1, 0, pi/2.0, 'R', round_decimal));
    T_yawlink_pitchbacklink = ...
        simplify(DH_Modified(-pi/2.0, 0, q2, 0, -pi/2.0, 'R', round_decimal));
    T_pitchendlink_maininsertionlink = ...
        simplify(DH_Modified(pi/2.0, 0, 0, q3, -L_rcc, 'P', round_decimal));
    T_maininsertionlink_toollink = ...
        simplify(DH_Modified(0, 0, q4, L_scopelen, 0, 'R', round_decimal));
    
    syms p_W_base_x p_W_base_y p_W_base_z;
    T_W_baselink = [
        1, 0, 0, p_W_base_x;
        0, 1, 0, p_W_base_y;
        0, 0, 1, p_W_base_z;
        ];
    
    T_W_yawlink           = simplify(T_W_baselink          * T_baselink_yawlink);
    T_W_pitchbacklink     = simplify(T_W_yawlink           * T_yawlink_pitchbacklink);
    T_W_maininsertionlink = simplify(T_W_pitchbacklink     * T_pitchendlink_maininsertionlink);
    T_W_toollink          = simplify(T_W_maininsertionlink * T_maininsertionlink_toollink);
    
    %% Jacobian
    J_W_v1(1:3, 1) = diff(T_W_yawlink(1:3, 4), q1);
    J_W_v1(1:3, 2) = zeros;
    J_W_v1(1:3, 3) = zeros;
    J_W_v1(1:3, 4) = zeros;

    J_W_v2(1:3, 1) = diff(T_W_pitchbacklink(1:3, 4), q1);
    J_W_v2(1:3, 2) = diff(T_W_pitchbacklink(1:3, 4), q2);
    J_W_v2(1:3, 3) = zeros;
    J_W_v2(1:3, 4) = zeros;

    J_W_v3(1:3, 1) = simplify(diff(T_W_maininsertionlink(1:3, 4), q1));
    J_W_v3(1:3, 2) = simplify(diff(T_W_maininsertionlink(1:3, 4), q2));
    J_W_v3(1:3, 3) = simplify(diff(T_W_maininsertionlink(1:3, 4), q3));

    J_W_v3(1:3, 4) = zeros;

    J_W_v4(1:3, 1) = simplify(diff(T_W_toollink(1:3, 4), q1));
    J_W_v4(1:3, 2) = simplify(diff(T_W_toollink(1:3, 4), q2));
    J_W_v4(1:3, 3) = simplify(diff(T_W_toollink(1:3, 4), q3));
    J_W_v4(1:3, 4) = simplify(diff(T_W_toollink(1:3, 4), q4));

    % --
    J_W_w1(1:3, 1) = zeros;
    J_W_w1(  3, 1) = 1;
    J_W_w1(1:3, 2) = zeros;
    J_W_w1(1:3, 3) = zeros;
    J_W_w1(1:3, 4) = zeros;
    
    J_W_w2 = J_W_w1;
    J_W_w2(  3, 2) = 1;
    
    J_W_w3 = J_W_w2;
    
    J_W_w4 = J_W_w3;
    J_W_w4(  3, 4) = 1;
    %% Mass Matrix Calculation
    J_W_v1_square = m1 * simplify(transpose(J_W_v1) * J_W_v1);
    J_W_v2_square = m2 * simplify(transpose(J_W_v2) * J_W_v2);
    J_W_v3_square = m3 * simplify(transpose(J_W_v3) * J_W_v3);
    J_W_v4_square = m4 * simplify(transpose(J_W_v4) * J_W_v4);
    
    % All w values are constant. Verify Calcuations.
    J_W_w1_square = (transpose(J_W_w1) * I1) * J_W_w1;
    J_W_w2_square = (transpose(J_W_w2) * I2) * J_W_w2;
    J_W_w3_square = (transpose(J_W_w3) * I3) * J_W_w3;
    J_W_w4_square = transpose(J_W_w4) * I4 * J_W_w4;
    
    syms M_syms(n);
    M_syms(n) = simplify( ...
        J_W_v1_square + J_W_v2_square + J_W_v3_square + J_W_v4_square + ...
        J_W_w1_square + J_W_w2_square + J_W_w3_square + J_W_w4_square);

%     4x4
    M = simplify(M_syms(n));
% Check Mass Matrix properties
% M12_21_equal = isequal((M(1, 2)), (M(2, 1)))
% M13_31_equal = isequal(simplify(M(1, 3)), simplify(M(3, 1)))
% 
% M14_41_equal = isequal(simplify(M(1, 4)), simplify(M(4, 1)))
% 
% M23_32_equal = isequal(simplify(M(2, 3)), simplify(M(3, 2)))
% M24_42_equal = isequal(simplify(M(2, 4)), simplify(M(4, 2)))
% 
% M34_43_equal = isequal(simplify(M(3, 4)), simplify(M(4, 3)))

    %% Centrifugal and Coriolis Vector V

%     % Christoffel Symbols
    syms V_syms(n);
    V_syms(n) = [ 0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0;];
    V = V_syms(n);
    qs = [q1; q2; q3; q4;];
    qds = [qd1; qd2; qd3; qd4;];

    % validate skew symmetry property. Refernce Stanford notes
    for i = 1:4
        for j = 1:4
            for k = 1:4
                val = 1/2 * (diff(M(i, j), qs(k)) * qds(k) + ...
                                 diff(M(i, k), qs(j)) * qds(j)- ...
                                 diff(M(j, k), qs(i)) * qds(i)) * qds(k);
                a(1, 1, k) = val;
            end
            V(i, j) = sum(a(:));
        end
    end

% Does not validate skew symmetry property. Reference Robot modeling and
% control book
%     for i = 1:4
%         for j = 1:4
%             for k = 1:4
%                 val = 1/2 * (diff(M(k, j), qs(j)) * qds(j) + ...
%                                  diff(M(k, i), qs(j)) * qds(j)- ...
%                                  diff(M(i, j), qs(k)) * qds(k)) * qds(i);
%                 a(1, 1, k) = val;
%             end
%             V(i, j) = sum(a(:));
%         end
%     end

    % validate skew symmetry property. Refernce Mathematical Introduction
    % to Robotic manipulation. Results matches with results w.r.t Standford
    % notes.
%     for i = 1:4
%         for j = 1:4
%             for k = 1:4
%                 val = 1/2 * (diff(M(i, j), qs(k)) * qds(k) + ...
%                                  diff(M(i, k), qs(j)) * qds(j)- ...
%                                  diff(M(k, j), qs(i)) * qds(i)) * qds(k);
%                 a(1, 1, k) = val;
%             end
%             V_(i, j) = sum(a(:));
%         end
%     end
%     isequal(V, V_)
%     %% Verify Skew Symmetry and Passivity Properites
% dM = diff(M, q1) * qd1 + diff(M, q2) * qd2 + diff(M, q3) * qd3 + diff(M, q4) * qd4;
% 
% N = simplify(dM - 2*V);
% 
% N12_21_equal = isequal(N(1, 2), -N(2, 1))
% N13_31_equal = isequal(N(1, 3), -N(3, 1))
% N14_41_equal = isequal(N(1, 4), -N(4, 1))
% 
% N23_32_equal = isequal(N(2, 3), -N(3, 2))
% N24_42_equal = isequal(N(2, 4), -N(4, 2))
% 
% N34_43_equal = isequal(N(3, 4), -N(4, 3))


qd_centrifugal = [
    qd1^2;
    qd2^2;
    qd3^2;
    qd4^2;
    ];
qd_coriolis = [
    qd1 * qd2;
    qd1 * qd3;
    qd1 * qd4;
    qd2 * qd3;
    qd2 * qd4;
    qd3 * qd4;
    ];


%% Calculation of Torque
    syms g;
    G_W = simplify(...
        - transpose(J_W_v1) * [ 0; 0; -m1 * g; ] ...
        - transpose(J_W_v2) * [ 0; 0; -m2 * g; ] ...
        - transpose(J_W_v3) * [ 0; 0; -m3 * g; ] ...
        - transpose(J_W_v4) * [ 0; 0; -m4 * g; ] ...
    );
qdds = [
    qdd1;
    qdd2;
    qdd3;
    qdd4;
    ];
    T = M * qdds + V + G_W

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
    