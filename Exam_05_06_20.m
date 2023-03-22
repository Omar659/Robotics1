clear all
close all
clc
% addpath("Functions/") % Linux
addpath("Functions\") % Windows

DH_table = [0 pi/2 2 pi;
            1 pi 0 3.40;
            1 pi 0 0.2;
            1 0 0 0.3]

A_01 =  dh_matrix(DH_table(1,1),DH_table(1,2),DH_table(1,3),DH_table(1,4));
A_12 =  dh_matrix(DH_table(2,1),DH_table(2,2),DH_table(2,3),DH_table(2,4));
A_23 =  dh_matrix(DH_table(3,1),DH_table(3,2),DH_table(3,3),DH_table(3,4));
A_34 =  dh_matrix(DH_table(4,1),DH_table(4,2),DH_table(4,3),DH_table(4,4));

A_02 = A_01 * A_12;
A_03 = A_01 * A_12 * A_23;
A_04 = A_01 * A_12 * A_23 * A_34;

O_A_i = [eye(4) A_01 A_02 A_03 A_04];

joint_types = ["r", "r", "r", "r", "ee"];

plot_robot_pose_lite(joint_types, DH_table, O_A_i, false)

%%
clear all
close all
clc

syms q1 q2 q3 l1 l2 real

r = [l1*cos(q1) + q3*cos(q1 +q2);
     l1*sin(q1) + q3*sin(q1 +q2);
     q1 + q2]

J = jacobian(r, [q1 q2 q3])

J_l =  subs(J, {l1}, {0.5})

J_q0 = subs(J_l, {q1, q2, q3}, {pi/2, 0, 3})
F = [0;1.5;-4.5]
tau = J_q0'*(-F)

J_s = subs(J_l, {q2}, {pi/2})

tau_s = J_s' * (-F)
simplify(det(J_s))

%%
clear all
close all
clc

syms q1 q2 q3 real

J_q = [-sin(q1)*(cos(q2) + cos(q2+q3)) -cos(q1)*(sin(q2) + sin(q2+q3)) -cos(q1)*sin(q2+q3);
       cos(q1)*(cos(q2) + cos(q2+q3)) -sin(q1)*(sin(q2) + sin(q2+q3)) -sin(q1)*sin(q2+q3);
       0 cos(q2) + cos(q2+q3) cos(q2+q3)]
simplify(det(J_q))
simplify(compute_minors(J_q))
J_qr1 = subs(J_q, {q1, q2, q3}, {0, pi/2, pi})
J_qr2 = subs(J_q, {q1, q2, q3}, {0, 0, 0})
J_qr3 = subs(J_q, {q1, q2, q3}, {0, pi, pi/2})
rank(J_qr1)
rank(J_qr2)
rank(J_qr3)

disp('qr1')
null(J_qr1)
orth(J_qr1,"skipnormalization")
disp('qr2')
null(J_qr2)
orth(J_qr2,"skipnormalization")
disp('qr3')
null(J_qr3)
orth(J_qr3,"skipnormalization")

simplify(det(J_qr2))
pinv(J_qr2) * [-1;1;0]
%simplify(compute_minors(J_q))
%%

A = [1 1 1;
     5 4 3;
     20 12 6]
B = [1;0;0]

inv(A)*B

syms qs qg v a ta tv T real
T_min_vi = ((qg - qs)/v)*(30*tv^4 -60*tv^3 + 30*tv^2)
T_min_ai = sqrt(((qg - qs)/a)*(120*tv^3 -180*tv^2 + 60*tv))

T_min_v1 = vpa(subs(T_min_vi, {qg, qs, v, tv}, {-pi/2, 0, 1, 0.5}),4)
T_min_v2 = vpa(subs(T_min_vi, {qg, qs, v, tv}, {pi/2, -pi/2, 2, 0.5}),4)
T_min_a1 = vpa(subs(T_min_ai, {qg, qs, a, tv}, {-pi/2, 0, 1.5, (3+sqrt(3))/6}),4)
T_min_a2 = vpa(subs(T_min_ai, {qg, qs, a, tv}, {pi/2, -pi/2, 2, (3-sqrt(3))/6}),4)

q_d_i = ((qg - qs)/T)*(30*tv^4 -60*tv^3 + 30*tv^2)
q_dd_i = ((qg - qs)/T^2)*(120*ta^3 -180*ta^2 + 60*ta)

vpa(subs(q_d_i, {qg, qs, T, tv}, {-pi/2, 0, 3.011, 0.5}),4)
vpa(subs(q_d_i, {qg, qs, T, tv}, {pi/2, -pi/2, 3.011, 0.5}),4)
vpa(subs(q_dd_i, {qg, qs, T, ta}, {-pi/2, 0, 3.011, (3+sqrt(3))/6}),4)
vpa(subs(q_dd_i, {qg, qs, T, ta}, {pi/2, -pi/2, 3.011, (3-sqrt(3))/6}),4)
%%
clear all
close all
clc

R1 = [1/sqrt(2) 0 1/sqrt(2);
      0 1 0;
      1/sqrt(2) 0 -1/sqrt(2)]

R2 = [-1/sqrt(3) -1/sqrt(2) -1/sqrt(6);
      -1/sqrt(3) 0 2/sqrt(6);
      -1/sqrt(3) 1/sqrt(2) -1/sqrt(6)]

R3 = [-sqrt(0.5) 1/sqrt(2) 0;
      sqrt(0.5) 1/sqrt(2) 0;
      0 0 -1]
det(R2)
R2'
inv(R2)
det(R3)
R3'
inv(R3)

%%
clear all
close all
clc

newton_method()
%%

