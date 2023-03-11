clear all
close all
clc

% addpath("Functions/") % Linux
addpath("Functions\") % Windows
syms alpha beta gamma real
simplify(euler_RPY([0 1 0],[1 0 0],[0 0 1],alpha, beta, gamma,true))
R = [0  1 0;
     0.5 0 sqrt(3)/2;
     sqrt(3)/2 0 -0.5]
Ry = rotation_around_r([0 1 0], alpha);
Rx = rotation_around_r([1 0 0], beta);
Rz = rotation_around_r([0 0 1], gamma);

Ryxz = Ry*Rx*Rz

sb = -R(2,3)
cbp = sqrt(-sb^2 + 1)
cbm = -sqrt(-sb^2 + 1)
Betap = atan2(sb, cbp)
Betam = atan2(sb, cbm)

sgp = R(2,1)/cbp
sgm = R(2,1)/cbm
csp = R(2,2)/cbp
csm = R(2,2)/cbm
Gammapp = atan2(sgp, csp)
Gammapm = atan2(sgp, csm)
Gammamp = atan2(sgm, csp)
Gammamm = atan2(sgm, csm)

sap = R(1, 3)/cbp
sam = R(1, 3)/cbm
cap = R(3,3)/cbp
cam = R(3,3)/cbm
Alphapp = atan2(sap, cap)
Alphapm = atan2(sap, cam)
Alphamp = atan2(sam, cap)
Alphamm = atan2(sam, cam)

Ral = rotation_around_r([0 1 0], 3.1416)
Rbe = rotation_around_r([1 0 0], -1.0472)
Rga = rotation_around_r([0 0 1], 1.5708)

vpa(Ral * Rbe * Rga,4)

Ral = rotation_around_r([0 1 0], 3.1416)
Rbe = rotation_around_r([1 0 0], pi/2)
Rga = rotation_around_r([0 0 1], 1.5708)
vpa(Ral * Rbe * Rga,4)

%%
clear all
close all
clc
R = [0  1 0;
     0.5 0 sqrt(3)/2;
     sqrt(3)/2 0 -0.5]
s_omega = S_omega([1;-1;0])

R_d = R*s_omega
%%
clear all
close all
clc
syms L0 L1 L2 L3 q1 q2 q3 real
% DH_table = [-pi/2 L1 L0 -pi/2;
%             pi/2 L2 0 -pi/2;
%             0 L3 0 0];
DH_table = [-pi/2 L1 L0 q1;
            pi/2 L2 0 q2;
            0 L3 0 q3];
% DH_table = [-pi/2 0.5 1 -pi/2;
%             pi/2 1 0 -pi/2;
%             0 1 0 0];
A_01 = dh_matrix(DH_table(1,2), DH_table(1,1), DH_table(1,3), DH_table(1,4))
A_12 = dh_matrix(DH_table(2,2), DH_table(2,1), DH_table(2,3), DH_table(2,4))
A_23 = dh_matrix(DH_table(3,2), DH_table(3,1), DH_table(3,3), DH_table(3,4))
A_02 = A_01*A_12
A_03 = simplify(A_01*A_12*A_23)
% O_A_i = [eye(4) A_01 A_02 A_03]
% joint_types = ['r', 'r', 'r', 'ee']
% plot_robot_pose_lite(joint_types, DH_table, O_A_i, false)
%%
clear all
close all
clc
syms a b q1 q2 q3 q4 real

r = [a*cos(q1) + q3*cos(q1 + q2) + b*cos(q1 + q2 + q4);
     a*sin(q1) + q3*sin(q1 + q2) + b*sin(q1 + q2 + q4);
     q1 + q2+ q4]
q = [q1 q2 q3 q4]
J = simplify(jacobian(r, q))
dete = simplify(det(J*J'))

Js = subs(J, {q3, q2}, {0, pi/2})
disp('determinante Js con pi/2')
simplify(det(Js*Js'))

rank(Js)
disp(simplify(orth(Js,"skipnormalization")))

null(Js)
%%
clear all
close all
clc
syms q1a q2a q1b q2b real
ra = [cos(q1a) + cos(q1a + q2a);
      sin(q1a) + sin(q1a + q2a)]

rb = [cos(q1b) + cos(q1b + q2b);
      sin(q1b) + sin(q1b + q2b)]
J_a = jacobian(ra, [q1a, q2a])
J_b = jacobian(rb, [q1b, q2b])

J_a = subs(J_a, {q1a, q2a}, {3*pi/4, -pi/2})
J_b = subs(J_b, {q1b, q2b}, {pi/2, -pi/2})

tau_a = J_a' * 10

ra = subs(ra, {q1a, q2a}, {3*pi/4, -pi/2});
rb = subs(rb, {q1b, q2b}, {pi/2, -pi/2});
ra = vpa(ra, 4)
rb = vpa(rb, 4)

