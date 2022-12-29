% Clear Variables 
clc
clear all
close all
%%
syms l1 l2 ... Distance of the center
a1 a2 ... Link Lengths
I1 I2 ... Moment of Inertia
theta1(t) theta2(t) ... Theta's are function of time
m1 m2 ... Masses of links
t1r t2r ... Replacement variables for subs
td1r td2r tdd1r tdd2r ... Replacement
t g;

th1 = theta1;
th2 = theta2;
%%
x1 = l1 * cos(th1);
y1 = l1 * sin(th1);
X1 = [x1; y1];

x2 = (a1*cos(th1)) + (l2*cos(th1+th2));
y2 = (a1*sin(th1)) + (l2*sin(th1+th2));
X2 = [x2; y2];
%%
XD1 = diff(X1,t);
XD2 = diff(X2,t);

v1s = simplify(XD1.' * XD1);
v2s = simplify(XD2.' * XD2);
k1 = (m1 * v1s * 0.5) + (I1 * (diff(th1, t))^2 * 0.5);
k2 = (m2 * v2s * 0.5) + (I2 * (diff(th1, t) + diff(th2, t))^2 * 0.5);
KE = simplify(k1 + k2);
p1 = m1 * g * y1;
p2 = m2 * g * y2;
PE = simplify(p1 + p2);

%%
L = KE - PE;
L = simplify(L);

aP = {theta1, theta2, diff(theta1(t), t), diff(theta2(t), t), diff(theta1(t), t, t), diff(theta2(t), t, t)};
sP = { t1r, t2r, td1r, td2r, tdd1r, tdd2r};

torq1 = diff(subs( diff(subs( L, aP, sP), td1r),sP, aP),t) - subs( diff(subs( L, aP, sP), t1r),sP, aP)
% torq2 = diff(subs( diff(subs( L, aP, sP), td2r),sP, aP),t) - subs( diff(subs( L, aP, sP), t2r),sP, aP);
% torques = [simplify(torq1); simplify(torq2)];

%%
% torques = torques(t); % convert symfunct to sym
% M11 = simplify(torques(1) - subs(torques(1),diff(theta1(t), t, t),0)) /diff(theta1(t), t, t);
% M12 = simplify(torques(1) - subs(torques(1),diff(theta2(t), t, t),0)) /diff(theta2(t), t, t);
% M21 = simplify(torques(2) - subs(torques(2),diff(theta1(t), t, t),0)) /diff(theta1(t), t, t);
% M22 = simplify(torques(2) - subs(torques(2),diff(theta2(t), t, t),0)) /diff(theta2(t), t, t);
% M = [M11 M12; M21 M22];
% 
% G = subs(torques, {diff(theta1(t), t, t), diff(theta2(t), t, t), diff(theta1(t), t), diff(theta2(t), t)}, {0,0,0,0});
% C1 = simplify(torques(1) - (M(1,:) * [diff(theta1(t), t, t) diff(theta2(t), t, t)].' + G(1)));
% C2 = simplify(torques(2) - (M(2,:) * [diff(theta1(t), t, t) diff(theta2(t), t, t)].' + G(2)));
% C = [C1; C2];