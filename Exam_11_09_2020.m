clear all
close all
clc

% addpath("Functions/") % Linux
addpath("Functions\") % Windows
syms t real

R = [cos(t) 0 sin(t);
     sin(t)^2 cos(t) -sin(t)*cos(t);
     -sin(t)*cos(t) sin(t) cos(t)^2]

R_d = diff(R, t)

s_w = simplify(R_d * R')

%%
clear all
close all
clc

syms

DH_table = [0 pi/2 0.892 0;
            -4.25 0 0 -pi/2;
            -3.92 0 0 0;
            0 -pi/2 1.093 pi/2;
            0 pi/2 0.9475 0;
            0 0 0.825 0]

A_01 = dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4));
A_12 = dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4));
A_23 = dh_matrix(DH_table(3,1), DH_table(3,2), DH_table(3,3), DH_table(3,4));
A_34 = dh_matrix(DH_table(4,1), DH_table(4,2), DH_table(4,3), DH_table(4,4));
A_45 = dh_matrix(DH_table(5,1), DH_table(5,2), DH_table(5,3), DH_table(5,4));
A_56 = dh_matrix(DH_table(6,1), DH_table(6,2), DH_table(6,3), DH_table(6,4));
A_02 = A_01 * A_12;
A_03 = A_01 * A_12 * A_23;
A_04 = A_01 * A_12 * A_23 * A_34;
A_05 = A_01 * A_12 * A_23 * A_34 * A_45;
A_06 = A_01 * A_12 * A_23 * A_34 * A_45 * A_56;
O_A_i = [eye(4) A_01 A_02 A_03 A_04 A_05 A_06];
O_A_i = vpa(O_A_i, 4);
joint_types = ['r', 'r', 'r', 'r', 'r', 'r', 'ee'];

plot_robot_pose_lite(joint_types,DH_table, O_A_i, false)

%%
clear all
close all
clc

syms q1a q2a real
w_P_a = [-2.5; 1; 0]
w_P_b = [1; 2; 0]
q_a = [q1a;q2a]

w_R_b = rotation_around_r([0 0 1], pi/6)
w_T_a = homogeneous_T(eye(3), w_P_a)
w_T_b = homogeneous_T(w_R_b, w_P_b)
p_ea = [cos(q1a) + cos(q1a+q2a);
        sin(q1a) + sin(q1a+q2a);
        0]
a_R_ea = rotation_around_r([0 0 1], q1a+q2a)
a_T_ea = homogeneous_T(a_R_ea, p_ea)
ea_R_eb = [-1 0 0;
            0 -1 0;
            0 0 1]
ea_T_eb = homogeneous_T(ea_R_eb, [0; 0; 0])

b_T_eb = simplify(inv(w_T_b)*w_T_a*a_T_ea*ea_T_eb)
subs(b_T_eb, {q1a, q2a}, {pi/3, -pi/2})

%%
clear all
close all
clc

syms l1 l2 l3 q1 q2 q3 real



J = [-sin(q1)*(l2*cos(q2)+l3*cos(q3)) -l2*cos(q1)*sin(q2) -l3*cos(q1)*sin(q3);
        cos(q1)*(l2*cos(q2)+l3*cos(q3)) -l2*sin(q1)*sin(q2) -l3*sin(q1)*sin(q3);
        0 l2*cos(q2) l3*cos(q3)]

simplify(det(J))
% minors = compute_minors(J)
J_s = subs(J,{q2, q3}, {pi/2, pi/2})
det(simplify(J_s))
rank(J_s)

orth(J_s, "skipnormalization")
orth(J_s', "skipnormalization")
null(J_s)
null(J_s')

%%
clear all
close all
clc

syms T dx real
A = [1 1 1;
     5 4 3;
     20 12 6]
C = [1 + (2*T)/dx;(2*T)/dx;0]
inv(A)*[C]
%%
clear all
close all
clc

DH_table = [
            1 pi/2 2 0;
            1 0 0 pi/4;
            1 0 0 pi/4;
            1 -pi/2 0 -pi/2]

A_01 = dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4));
A_12 = dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4));
A_23 = dh_matrix(DH_table(3,1), DH_table(3,2), DH_table(3,3), DH_table(3,4));
A_34 = dh_matrix(DH_table(4,1), DH_table(4,2), DH_table(4,3), DH_table(4,4));
% A_45 = dh_matrix(DH_table(5,1), DH_table(5,2), DH_table(5,3), DH_table(5,4));

A_02 = A_01*A_12;
A_03 = A_01*A_12*A_23;
A_04 = A_01*A_12*A_23*A_34;
% A_05 = A_01*A_12*A_23*A_34*A_45;

O_A_i = [eye(4) A_01 A_02 A_03 A_04]% A_05]

joint_types = ['p','r','r','r','ee']

plot_robot_pose_lite(joint_types,DH_table,O_A_i,false)
%%

clear all
close all
clc

syms t r real

p_t = [r*cos(t)^2;r*cos(t)*sin(t);0]
p_t_p = diff(p_t, t)
t_t = simplify(p_t_p / norm(p_t_p))
t_t_p = diff(t_t,t)
n_s = simplify(t_t_p/norm(t_t_p))

size(t_t)
size(n_s)
b_s = simplify(cross(t_t,n_s))
%%
clear all
close all
clc

syms alpha beta gamma real

R_y = rotation_around_r([0 1 0], alpha)
R_x = rotation_around_r([1 0 0], beta)
R_y1= rotation_around_r([0 1 0], gamma)
R_yx = R_y * R_x
w = [R_yx(1,2) R_y(1,1) 0;
     R_yx(2,2) R_y(2,1) 1;
     R_yx(3,2) R_y(3,1) 0]
simplify(det(w))
%%
