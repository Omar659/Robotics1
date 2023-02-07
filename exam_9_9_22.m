addpath("Functions/") % Linux
% addpath("Functions\") % Windows

%%
clear all
close all
clc

DH_table = [0.75,  pi/2,     6.48,   pi;
            6.4,  pi,     0,     pi/2;
            2.25,  -pi/2,     0,  0;
            0,  -pi/2,     7,  0;
            0,  -pi/2,     0,  pi;
            0,  0,     0.75,      pi];
joint_types = ["r" "r" "r" "r" "r" "r" "ee"];

A_01 = dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4));
A_12 = dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4));
A_23 = dh_matrix(DH_table(3,1), DH_table(3,2), DH_table(3,3), DH_table(3,4));
A_34 = dh_matrix(DH_table(4,1), DH_table(4,2), DH_table(4,3), DH_table(4,4));
A_45 = dh_matrix(DH_table(5,1), DH_table(5,2), DH_table(5,3), DH_table(5,4));
A_56 = dh_matrix(DH_table(6,1), DH_table(6,2), DH_table(6,3), DH_table(6,4));
A_02 = A_01*A_12;
A_03 = A_01*A_12*A_23;
A_04 = A_01*A_12*A_23*A_34;
A_05 = A_01*A_12*A_23*A_34*A_45;
A_06 = A_01*A_12*A_23*A_34*A_45*A_56;
O_A_i = [eye(4) A_01 A_02 A_03 A_04 A_05 A_06];
O_A_i = vpa(O_A_i,4);
DH_table = vpa(DH_table,4);
figure(1);
clf
daspect([1 1 1])
plot_robot_pose_lite(joint_types, DH_table, O_A_i, false)

%%
clear all
close all
clc


syms q1 q2 q3 q1d q2d q3d L

f_r = [q2*cos(q1) + L*cos(q1+q3);
       q2*sin(q1) + L*sin(q1+q3);
       q1 + q3]

J = simplify(jacobian(f_r, [q1 q2 q3]))

det_J = simplify(det(J))

J_s = simplify(subs(J, {q2}, {0}))

rank([J_s(:, 1) J_s(:, 2)])

null_J = simplify(null(J_s))

J_particular = simplify(subs(J, {q1, q2, q3, L}, {pi/2, 1, 0, 1}))

J_dot = [-L*cos(q1 + q3)*(q1d + q3d)-q2d*sin(q1)-q2*cos(q1)*q1d, -sin(q1)*q1d, -L*cos(q1 + q3)*(q1d+q3d);
         -L*sin(q1 + q3)*(q1d + q3d)+q2d*cos(q1)-q2*sin(q1)*q1d, cos(q1)*q1d -L*sin(q1 + q3)*(q1d+q3d);
         0 0 0]

J_dot_subs = simplify(subs(J_dot, {q1, q2, q3, L, q1d, q2d, q3d}, {pi/2, 1, 0, 1, 1, -1, -1}))
h_qqd = J_dot_subs*[1; -1; -1]

%%


clear all
close all
clc












