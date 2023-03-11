addpath("Functions/") % Linux
% addpath("Functions\") % Windows

%%
clear all
close all
clc

syms q1 q2 q3 a L real
px = q1 + L*cos(q2) + L*cos(q2+q3);
py = L*sin(q2) + L*sin(q2+q3);
alpha = q2 +q3;
r = [px;py;alpha]
q = [q1;q2;q3]
J = simplify(jacobian(r, q))
J_s = subs(J, {q3,q2}, {0, pi/2})
null(J_s)
orth(J_s)
psinv = simplify(pinv(J_s))
q_d =simplify(psinv * [1/2;0;1/2])
null(J_s')

s2 = (0.7 - 0.5*sin(pi/3))/0.5
c2 = sqrt(1-s2^2)
q_2 = atan2(s2,c2)

q_3 = (pi/3) - q_2

q_1 = 0.3 - 0.5*cos(q_2) - 0.5*cos(q_2 + (pi/3) - q_2)

vpa(subs(px, {L, q1, q2, q3}, {0.5, q_1, q_2, q_3}),4)
vpa(subs(py, {L, q1, q2, q3}, {0.5, q_1, q_2, q_3}),4)
vpa(subs(alpha, {L, q1, q2, q3}, {0.5, q_1, q_2, q_3}),4)
%%
syms tau real
r_i = [2*L; 0; pi/4];
r_f = [2*L; 0; -pi/4];
L = vpa(norm(r_f - r_i),4);
s2 = (0 - sym('L')*sin(pi/4))/sym('L');
c2 = sqrt(1-s2^2);
q_2 = atan2(s2,c2);

q_3 = (pi/4) - q_2;

q_1 = 2*sym('L') - sym('L')*cos(q_2) - sym('L')*cos(q_2 + (pi/4) - q_2);
q_in = [q_1; q_2; q_3]

s2 = (0 - sym('L')*sin(-pi/4))/sym('L');
c2 = sqrt(1-s2^2);
q_2 = atan2(s2,c2);

q_3 = (-pi/4) - q_2;

q_1 = 2*sym('L') - sym('L')*cos(q_2) - sym('L')*cos(q_2 + (-pi/4) - q_2);
q_fin = [q_1; q_2; q_3]
d_q = q_fin - q_in

x = linspace(q_in(2), q_fin(2), 100)
x = subs(x, {sym('L')}, {1})
y = -pi/4 + pi/2*(2*tau^3 + 3*tau^2)
y = subs(y, {tau}, x)
plot(y(1,:))
x = linspace(q_in(3), q_fin(3), 100)
x = subs(x, {sym('L')}, {1})
y = pi/2 - pi/2*(6*tau^2 + 6*tau)
y = subs(y, {tau}, x)
plot(y(1,:))

%%
clear all
close all
clc
syms B A C D a1 a2 a3 a4 q1 q2 q3 q4 q1_d q2_d q3_d q4_d real
DH_table = [1 -pi/2 2 pi/2;
            3 0 0 -pi/4;
            0.7 -pi/2 0 -pi/6;
            0 0 4 0]
A_01 = dh_matrix(DH_table(1, 1), DH_table(1, 2), DH_table(1, 3), DH_table(1, 4));
A_12 = dh_matrix(DH_table(2, 1), DH_table(2, 2), DH_table(2, 3), DH_table(2, 4));
A_23 = dh_matrix(DH_table(3, 1), DH_table(3, 2), DH_table(3, 3), DH_table(3, 4));
A_34 = dh_matrix(DH_table(4, 1), DH_table(4, 2), DH_table(4, 3), DH_table(4, 4));
A_02 = A_01*A_12;
A_03 = A_01*A_12*A_23;
A_04 = A_01*A_12*A_23*A_34;
O_A_i = [eye(4) A_01 A_02 A_03 A_04];
joint_types = ['r', 'r', 'r', 'p', "ee"]
% plot_robot_pose_lite(joint_types, DH_table, O_A_i, false)

DH_table = [B -pi/2 A q1;
            C 0 0 q2;
            D -pi/2 0 q3;
            0 0 q4 0]
A_01 = dh_matrix(DH_table(1, 1), DH_table(1, 2), DH_table(1, 3), DH_table(1, 4));
A_12 = dh_matrix(DH_table(2, 1), DH_table(2, 2), DH_table(2, 3), DH_table(2, 4));
A_23 = dh_matrix(DH_table(3, 1), DH_table(3, 2), DH_table(3, 3), DH_table(3, 4));
A_34 = dh_matrix(DH_table(4, 1), DH_table(4, 2), DH_table(4, 3), DH_table(4, 4));
A_02 = A_01*A_12;
A_03 = A_01*A_12*A_23;
A_04 = A_01*A_12*A_23*A_34
fr = [simplify(A_04(1:3,4)); q1+q2+q3]
J = simplify(jacobian(fr, [q1;q2;q3;q4]))
J_red = simplify(subs(J, {B, D}, {0, 0}))
simplify(det(J_red))
J_red_s = subs(J_red, {q4, q2}, {0, pi/2})
simplify(det(J_red_s))
rank(J_red_s)
simplify(orth(J_red_s))
simplify(pinv(J_red_s))*[0;0;sin(q3);0]

%q_dd = inv(J)*
% 
% DH_table = [1.557,     0,     3,     0;
%             1.25,  -pi/2,     0, -pi/2;
%             0,     -pi/2,     0,     0;
%             0,     -pi/2, 3.115,    pi;
%             0,     -pi/2,     0, -pi/2;
%             0,     -pi/2,  3.12,    pi;
%             0,     -pi/2,     0, -pi/2;
%             0,         0,     0,    pi];
% joint_types = ["p" "r" "r" "r" "r" "r" "r" "r" "ee"];
% 
% A_01 = dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4));
% A_12 = dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4));
% A_23 = dh_matrix(DH_table(3,1), DH_table(3,2), DH_table(3,3), DH_table(3,4));
% A_34 = dh_matrix(DH_table(4,1), DH_table(4,2), DH_table(4,3), DH_table(4,4));
% A_45 = dh_matrix(DH_table(5,1), DH_table(5,2), DH_table(5,3), DH_table(5,4));
% A_56 = dh_matrix(DH_table(6,1), DH_table(6,2), DH_table(6,3), DH_table(6,4));
% A_67 = dh_matrix(DH_table(7,1), DH_table(7,2), DH_table(7,3), DH_table(7,4));
% A_78 = dh_matrix(DH_table(8,1), DH_table(8,2), DH_table(8,3), DH_table(8,4));
% A_02 = A_01*A_12;
% A_03 = A_01*A_12*A_23;
% A_04 = A_01*A_12*A_23*A_34;
% A_05 = A_01*A_12*A_23*A_34*A_45;
% A_06 = A_01*A_12*A_23*A_34*A_45*A_56;
% A_07 = A_01*A_12*A_23*A_34*A_45*A_56*A_67;
% A_08 = A_01*A_12*A_23*A_34*A_45*A_56*A_67*A_78;
% O_A_i = [eye(4) A_01 A_02 A_03 A_04 A_05 A_06 A_07 A_08];
% O_A_i = vpa(O_A_i,4);
% DH_table = vpa(DH_table,4);
% figure(1);
% clf
% daspect([1 1 1])
% plot_robot_pose(joint_types, DH_table, O_A_i, false)
% 
% %%
% clear all
% close all
% clc
% 
% syms phi a1 a2 alp2 alp3 alp4 alp5 alp6 alp7 d4 d6 q1 q2 q3 q4 q5 q6 q7 q8 real
% w_R_0 = rotation_around_r([0 0 1], -pi/4)
% w_p_w0 = [1.5; -4.5; 0.3]
% w_T_0 = homogeneous_T(w_R_0, w_p_w0)
% 
% DH_table = [a1,     0, q1,  0;
%             a2,  -pi/2,  0, q2;
%             0,   -pi/2,  0, q3;
%             0,   -pi/2, d4, q4;
%             0,   pi/2,  0, q5;
%             0,   -pi/2, d6, q6;
%             0,   -pi/2,  0, q7;
%             0,      0,  0, q8];
% 
% A_01 = dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4));
% A_12 = dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4));
% A_23 = dh_matrix(DH_table(3,1), DH_table(3,2), DH_table(3,3), DH_table(3,4));
% A_34 = dh_matrix(DH_table(4,1), DH_table(4,2), DH_table(4,3), DH_table(4,4));
% A_45 = dh_matrix(DH_table(5,1), DH_table(5,2), DH_table(5,3), DH_table(5,4));
% A_56 = dh_matrix(DH_table(6,1), DH_table(6,2), DH_table(6,3), DH_table(6,4));
% A_67 = dh_matrix(DH_table(7,1), DH_table(7,2), DH_table(7,3), DH_table(7,4));
% A_78 = dh_matrix(DH_table(8,1), DH_table(8,2), DH_table(8,3), DH_table(8,4));
% 
% p_0W = simplify(w_T_0*(A_01*(A_12*(A_23*(A_34*(A_45*[0; 0; d6; 1]))))))
% 
% %% 
% clear all
% close all
% clc
% 
% syms q1 q2 q3 q4 q1_d q2_d q3_d q4_d real
% f_r = [q2*cos(q1) + q4*cos(q1+q3);
%        q2*sin(q1) + q4*sin(q1+q3);
%        q1 + q3]
% [J, r_d] = analityc_J(f_r, [q1; q2; q3; q4], [q1_d; q2_d; q3_d; q4_d]);
% J = simplify(J)
% r_d = simplify(r_d);
% singular_cases = simplify(det(J*J'))
% 
% singular_q = [q1; 0; 0; q4]
% regular_q = [q1; q2; q3; q4]
% 
% J_regular_q = subs(J, {q1 q2 q3 q4}, {regular_q(1), regular_q(2), regular_q(3), regular_q(4)})
% n_regular_J = simplify(null(J_regular_q))
% check = simplify(J_regular_q*n_regular_J)
% 
% J_singular_q = subs(J, {q1 q2 q3 q4}, {singular_q(1), singular_q(2), singular_q(3), singular_q(4)})
% n_singular_J = simplify(null(J_singular_q))
% rank(J_singular_q)
% check = J_singular_q*(n_singular_J(:, 1) + n_singular_J(:, 2))
% 
% range_singular_J = simplify(null(J_singular_q'))
% 
% J_T = J'
% 
% J_T_regular_q = subs(J_T, {q1 q2 q3 q4}, {regular_q(1), regular_q(2), regular_q(3), regular_q(4)})
% n_regular_J_T = simplify(null(J_T_regular_q))
% 
% J_T_singular_q = subs(J_T, {q1 q2 q3 q4}, {singular_q(1), singular_q(2), singular_q(3), singular_q(4)})
% n_singular_J_T = simplify(null(J_T_singular_q))
% check = J_T_singular_q*n_singular_J_T
% 
% %%
% clear all
% close all
% clc
% 
% syms s r h V A s_d s_dd real
% 
% C = [0; 0; r];
% p_s = [r*cos(s);
%        h*s;
%        r*sin(s)];
% % p_s = simplify(p_s + C);
% % s_plot = linspace(0, 1, 100);
% % p_s_plot = subs(p_s, {s}, {s_plot})
% % plot3(p_s_plot(1,:), p_s_plot(2,:), p_s_plot(3,:))
% % grid on
% [t_s, n_s, b_s, k_s, tau_s] = frenet_frame(p_s, s);
% t_s
% n_s
% p_s_p = simplify(diff(p_s))
% p_s_s = simplify(diff(p_s_p))
% 
% p_d = simplify(norm(p_s_p)*s_d)
% p_dd_t = simplify((p_s_s'*t_s)*s_d^2 + (p_s_p'*t_s)*s_dd)
% p_dd_n = simplify((p_s_s'*n_s)*s_d^2 + (p_s_p'*n_s)*s_dd)
% 
% % r = 0.4;
% % h = 0.3;
% % V = 2;
% % A = 4.5;