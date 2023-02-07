addpath("Functions/") % Linux
% addpath("Functions\") % Windows

%%
clear all
close all
clc


DH_table = [0,  pi/2,     1,   pi/4;
            0,  pi/2,     2,   pi/2;
            1,  pi/2,     0,   pi/6];
joint_types = ["r" "p" "r" "ee"];

A_01 = dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4));
A_12 = dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4));
A_23 = dh_matrix(DH_table(3,1), DH_table(3,2), DH_table(3,3), DH_table(3,4));
A_02 = A_01*A_12;
A_03 = A_01*A_12*A_23;
O_A_i = [eye(4) A_01 A_02 A_03];
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

syms q1 q2 q3 a3 d1 d2 th2

DH_table = [0,  pi/2,     d1,   q1;
            0,  pi/2,     q2,   pi/2;
            a3, pi/2,     0,   q3];

A_01 = simplify(dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4)))
A_12 = simplify(dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4)))
A_23 = simplify(dh_matrix(DH_table(3,1), DH_table(3,2), DH_table(3,3), DH_table(3,4)))
A_03 = simplify(A_01*(A_12*(A_23*[0;0;0;1])))

%%

clear all
close all
clc

syms q1 q2 q3 a3 d1 d2 th2 q1d q2d q3d real

f_r = [sin(q1)*(q2 + a3*sin(q3));
       -cos(q1)*(q2 + a3*sin(q3));
       d1 + a3*cos(q3)]
J = simplify(jacobian(f_r, [q1 q2 q3]))

det_J = simplify(det(J))

J1 = simplify(subs(J, {q3}, {0}))
J2 = simplify(subs(J, {q3}, {pi}))

rank_J1 = rank(J1)
rank_J2 = rank(J2)

N_J1 = simplify(null(J1))
N_J2 = simplify(null(J2))

mob_lost_J1 = simplify(null(J1'))
mob_lost_J2 = simplify(null(J2'))

%%

clear all
close all
clc

syms px py l1 l2 real

c2 = (px^2 + py^2 - l1^2 - l2^2)/(2*l1*l2);
s2 = simplify(sqrt(1-c2^2));
q2 = atan2(s2, c2);
q1 = atan2(py, px) - atan2(l2*sin(q2), l1+ l2*cos(q2));

q2_in = vpa(simplify(subs(q2, {px, py, l1, l2}, {2+1/sqrt(2), 1/sqrt(2), 2, 1})),4)
q1_in = vpa(simplify(subs(q1, {px, py, l1, l2}, {2+1/sqrt(2), 1/sqrt(2), 2, 1})),4)
q2_fin = vpa(simplify(subs(q2, {px, py, l1, l2}, {3/sqrt(2), -1/sqrt(2), 2, 1})),4)
q1_fin = vpa(simplify(subs(q1, {px, py, l1, l2}, {3/sqrt(2), -1/sqrt(2), 2, 1})),4)









