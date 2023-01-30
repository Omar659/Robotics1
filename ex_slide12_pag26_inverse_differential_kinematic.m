clear all
close all
clc
% da fixare
% addpath("Functions/") % Linux
addpath("Functions\") % Windows

syms q1_a q2_a q1_b q2_b real
q_a = [3*pi/4; -pi/2];
q_b = [pi/2; -pi/2];

DH_table = [1 0 0 q1_a;
            1 0 0 q2_a];

A_01_a = dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4));
A_12_a = dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4));
A_02_a = A_01_a * A_12_a;

% joint_types = ['r', 'r', "ee"];
% O_A_i_a = [eye(4) A_01_a A_02_a];
%plot_robot_pose(joint_types, DH_table, O_A_i)

DH_table = [1 0 0 q1_b;
            1 0 0 q2_b];

O_b = A_02_a(1:3, 4) + [1;1;0];
b_T_a = [-1 0 0 O_b(1);
         0 -1 0 O_b(2);
         0 0 1 O_b(3);
         0 0 0 1];

A_01 = dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4));
A_12 = dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4));
b_A_01 = b_T_a * A_01;
b_A_02 = b_T_a * A_01 * A_12;

joint_types = ['r', 'r', "ee"];
O_A_i = [b_T_a b_A_01 b_A_02];
O_A_i = subs(O_A_i, {q1_a, q2_a, q1_b, q2_b}, {q_a(1), q_a(2), q_b(1), q_b(2)});
DH_table = subs(DH_table, {q1_a, q2_a, q1_b, q2_b}, {q_a(1), q_a(2), q_b(1), q_b(2)});
plot_robot_pose(joint_types, DH_table, O_A_i)

EE_a = simplify(A_02_a(1:3, 4))
J_a = simplify(jacobian(EE_a, [q1_a; q2_a]))
F_a = simplify(10*A_02_a(1:3, 1))
tau_a = simplify(compute_tau(J_a, F_a))

EE_b = simplify(b_A_02(1:3, 4))
J_b = simplify(jacobian(EE_b, [q1_b; q2_b]))
F_b = -F_a
tau_b = simplify(compute_tau(J_b, F_b))

tau_a = subs(tau_a, {q1_a, q2_a}, {q_a(1), q_a(2)})
tau_b = subs(tau_b, {q1_a, q2_a, q1_b, q2_b}, {q_a(1), q_a(2), q_b(1), q_b(2)})
% b_F_b = b_R_a * a_F_b = b_R_a * (-a_F_a) = a_F_a
b_T_a(1:3, 1:3)'*F_b
F_a