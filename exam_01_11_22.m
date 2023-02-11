addpath("Functions/") % Linux
% addpath("Functions\") % Windows

%%
clear all
close all
clc
syms q1 q2 q3 L M K real
phi = -atan(L/M);
DH_table = [K,               -pi/2, 0,  q1;
            0,                pi/2, q2, 0;
            sqrt(L^2 + M^2), 0, 0,  q3];
joint_types = ["r" "p" "r" "ee"];

A_01 = dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4));
A_12 = dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4));
A_23 = dh_matrix(DH_table(3,1), DH_table(3,2), DH_table(3,3), DH_table(3,4));

A_03 = simplify(A_01*A_12*A_23)

compute_theta(A_03)

%%
clear all
close all
clc
syms q1 q2 q3 D L M K px py pz real

p_x = D*cos(q1 + q3) + K*cos(q1) - q2*sin(q1);
p_y = D*sin(q1 + q3) + K*sin(q1) + q2*cos(q1);
phi = q1 + q3;

f_r = [p_x; p_y; phi]

J = simplify(jacobian(f_r, [q1 q2 q3]))

det_J = simplify(det(J))

J_s = subs(J, {q2}, {0})

range_J_s = J_s(:, 2:3)


%%
clear all
close all
clc


n_turn = 700/360
ceil(log2(n_turn*30))-1
ceil(log2(360/(0.02*30)))









