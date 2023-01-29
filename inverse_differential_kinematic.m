clear all
close all
clc

% addpath("Functions/") % Linux
addpath("Functions\") % Windows

syms q1 q2 v1 v2 real
J = [-2*sin(q1) -sin(q1);
     2*cos(q1) cos(q1)];
v = [v1; v2];

% pseudoinverse method 
[J_pinv, q_dot, v_feasible] = pinv_J(J, v);
display(J)
J_pinv = simplify(J_pinv)
q_dot = simplify(q_dot)
v_feasible = simplify(v_feasible)

% case 1 --> q1 = pi/6
J_1 = subs(J, {q1}, {pi/6})
J_pinv_1 = subs(J_pinv, {q1}, {pi/6})
q_dot_1 = subs(q_dot, {q1, v1, v2}, {pi/6, -0.5, 0})
% v_1 in this case differs from v_feasible_1 beacause v_1 is not in 
% the range of J_1
v_1 = subs(v, {v1, v2}, {-0.5, 0})
v_feasible_1 = subs(v_feasible, {q1, v1, v2}, {pi/6, -0.5, 0})
range_J_1 = orth(J_1) % range J_1
disp('Rank of J_1')
rank(range_J_1)
disp('if the rank of the matrix obtained combining range_J_1')
disp('and v_1 is the same of the rank of range_J_1,')
disp('then v_1 belong to the range of J_1')
rank([range_J_1 v_1])
disp('v_feasible_1 should be in the range of J_1,') 
disp('in fact the rank is the same of J_1')
rank([range_J_1 v_feasible_1])

% case 2 --> q1 = pi/2
J_2 = subs(J, {q1}, {pi/2})
J_pinv_2 = subs(J_pinv, {q1}, {pi/2})
q_dot_2 = subs(q_dot, {q1, v1, v2}, {pi/2, -0.5, 0})
% v_2 in this case differs from v_feasible_2 beacause v_2 is not in 
% the range of J_2
v_2 = subs(v, {v1, v2}, {-0.5, 0})
v_feasible_2 = subs(v_feasible, {q1, v1, v2}, {pi/2, -0.5, 0})
range_J_2 = orth(J_2) % range J_2
disp('Rank of J_2')
rank(range_J_2)
disp('if the rank of the matrix obtained combining range_J_2')
disp('and v_2 is the same of the rank of range_J_2,')
disp('then v_2 belong to the range of J_2')
rank([range_J_2 v_2])
disp('v_feasible_2 should be in the range of J_2,')
disp('in fact the rank is the same of J_2')
rank([range_J_2 v_feasible_2])
