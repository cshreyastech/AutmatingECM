% Clear Variables 
clc
clear all
close all
%%
% syms x(t) m k
% T = m/2*diff(x(t),t)^2;
% V = k/2*x(t)^2;
% 
% L = T - V
% 
% D1 = diff(L,diff(x(t),t))
%%
% syms x y
% coeffs_x = collect(x^2*y + y*x - x^2 - 2*x,x)
% coeffs_y = collect(x^2*y + y*x - x^2 - 2*x,y)
%%
% s = tf('s');
% H = exp(-2.5*s)/(s+12);
% [num,den] = tfdata(H,'v')
%%
syms x y z
q = x^3 + 2*x^2*y + 3*x*y^2 + 4*y^3 + x*y
[cxy, txy] = coeffs(q, [x,y])
[cyx, tyx] = coeffs(x^3 + 2*x^2*y + 3*x*y^2 + 4*y^3, [y,x])
[cxz, txz] = coeffs(x^3 + 2*x^2*y + 3*x*y^2 + 4*y^3, [x,z])

hasSymType(q, 'symfunDependingOn', [x])
%%
% syms f(x,y,z)
% % g = f + x*y + pi
% g = x*y + pi
% TF = hasSymType(g,'symfunOf',x)
% TF = hasSymType(g,'symfunOf',[x y z])
% TF = hasSymType(g,'symfunOf',[x y])
% TF = hasSymType(g,'symfunDependingOn',[y x])
% TF = hasSymType(g,'symfunDependingOn',[ x y])