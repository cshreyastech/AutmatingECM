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
s = tf('s');
H = exp(-2.5*s)/(s+12);
[num,den] = tfdata(H,'v')

