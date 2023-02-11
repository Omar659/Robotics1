addpath("Functions/") % Linux
% addpath("Functions\") % Windows

%%
clear all
close all
clc



syms a1 a2 a3 ph real


XYX_euler = simplify(euler_RPY([0 1 0], [1 0 0], [0 1 0], a1, a2, a3, true))
O_R_f = [0 sin(ph) cos(ph);
         0 cos(ph) -sin(ph);
         -1 0 0]

Ra = vpa(simplify(subs(XYX_euler, {a1 a2 a3}, {pi/4, -pi/4, (2/3)*pi})),4)
Rb = vpa(simplify(subs(O_R_f, {ph}, {pi/3})), 4)

Rr = vpa(Ra'*Rb, 4)

[n, th_ab_p, th_ab_n] = compute_theta(Rr);

th_ab_p = vpa(th_ab_p, 4)
th_ab_n = vpa(th_ab_n, 4)

r = vpa(compute_r(Rr, th_ab_p), 4)

w = vpa(1.1*r, 4)

T = vpa(th_ab_p./w, 4)








