addpath("Functions/") % Linux
% addpath("Functions\") % Windows

%%
clear all
close all
clc



syms q1 q2 q3 L real
f_r = [q1 + L*cos(q2) + L*cos(q2+q3);
       L*sin(q2) + L*sin(q2+q3);
       q2 + q3]

J = simplify(jacobian(f_r, [q1 q2 q3]))
det_J = simplify(det(J))

J_s = subs(J, {q2}, {pi/2})

null_J_s = null(J_s)

range_J_s = [J_s(:,1), J_s(:,3)]

r1 = J_s*[-1; 1; 0]

r2 = simplify(null(J_s'))

null(J_s')

p_x = f_r(1)
p_y = f_r(2)
al_ = f_r(3)

syms px py al real
p_x = subs(p_x, {q2 + q3}, {al})
p_y = subs(p_y, {q2 + q3}, {al})
simplify(expand((p_x - L*cos(al))^2 + (p_y - L*sin(al))^2))






