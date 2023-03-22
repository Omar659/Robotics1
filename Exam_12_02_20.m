clear all
close all
clc
% addpath("Functions/") % Linux
addpath("Functions\") % Windows

syms la lb q1 q2 q3 q4 q1_d q2_d q3_d q4_d real
DH_table = [0 0 q1 0;
            0 -pi/2 0 q2;
            0 pi/2 la q3;
            lb 0 0 q4]

A_01 = dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4));
A_12 = dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4));
A_23 = dh_matrix(DH_table(3,1), DH_table(3,2), DH_table(3,3), DH_table(3,4));
A_34 = dh_matrix(DH_table(4,1), DH_table(4,2), DH_table(4,3), DH_table(4,4));

A_02 = A_01*A_12;
A_03 = A_01*A_12*A_23;
A_04 = A_01*A_12*A_23*A_34;
%O_A_i = [eye(4) A_01 A_02 A_03 A_04];

joint_variables = ["p", "r", "r", "r", "ee"]

%plot_robot_pose_lite(joint_variables, DH_table, O_A_i, false)
p = simplify(A_04(1:3,4))

fq_p = subs(p, {la,lb,q1,q2,q3,q4}, {0.5, 0.75, 0, 0, 0, 0})
fq_s = subs(p, {la,lb,q1,q2,q3,q4}, {0.5, 0.75, 1, 0, -pi/2, pi/2})

J_p = simplify(jacobian(subs(p, {la,lb}, {0.5, 0.75}), [q1;q2;q3;q4]))

tau = simplify(J_p' * [0; 0; -9.8])

tau_q_p = vpa(subs(tau, {q1,q2,q3,q4}, {0,0,0,0}),4)
tau_q_s = vpa(subs(tau, {q1,q2,q3,q4}, {1,0,-pi/2,pi/2}),4)

[v_E, w_E, J] = geometric_J(DH_table,joint_variables, [q1_d;q2_d;q3_d;q4_d;]);
simplify(v_E)
simplify(w_E)
J = (simplify(J))
J_a = J(4:end, :)
J_as = subs(J_a, {q3}, {0})
null(J_as)
J_as*[1;0;0;0]

J_as = subs(J_a, {q3}, {pi})
null(J_as)
