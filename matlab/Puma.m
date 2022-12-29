

function Puma
%%

close all
clc            % clear the command window
clear all      % clear all variables
format compact % condense command window output
format long g  % +, bank, hex, long, rat, short, short g, short eng
%%




function T_N_Nplus1 = DH_Modified(type, alpha, a, theta, d, offset)
    if(type == 'R')
        theta = theta + offset;
    elseif(type == 'P')
        d = d + offset;
    else
        fprintf("Invalid Joint type");
    end

    ca = cos(alpha);
    sa = sin(alpha);

    ct = cos(theta);
    st = sin(theta);

    
    T_N_Nplus1 = [ 
        ct, -st * ca,  st * sa, a * ct;
        st,  ct * ca, -ct * sa, a * st;
         0,       sa,       ca,      d;
         0,        0,        0,      1;
         ];
end