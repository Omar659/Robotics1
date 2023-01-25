clear all
close all
clc

% addpath("Functions/") % Linux
addpath("Functions\") % Windows

syms th ph ps th_d ph_d ps_d real

S = S_omega([ph_d; th_d; ps_d]);
display(S)

S = S_omega([ph_d; 0; 0]);
display(S)

R = rotation_around_r([1 0 0], ph);
display(R)

R_dot = compute_R_dot(S, R);
display(R_dot)

S = simplify(compute_S(R_dot, R));
display(S)

syms alp bet gam real
% Jai's example
T_RPY = simplify(derivative_RPY(1, 3, 2, [alp bet gam]));
display(T_RPY)
% Slide example
T_RPY = simplify(derivative_RPY(1, 2, 3, [alp bet gam]));
display(T_RPY)


% Example slide 11 pag 13
syms l1 l2 q1 q2 q1_d q2_d real
% r = [p; phi] = fr(q) direct kinematic
fr = [l1*cos(q1) + l2*cos(q1 + q2);
      l1*sin(q1) + l2*sin(q1 + q2);
      q1 + q2];
q = [q1; q2];
q_d = [q1_d; q2_d];
J = jacobian(fr, q);
display(J)
r_d = simplify(analityc_J(fr, q, q_d));
display(r_d)

% Example slide 11 pag 14
syms q1 q2 q3 d1 q1_d q2_d q3_d real
% r = [p; phi] = fr(q) direct kinematic
fr = [q3*cos(q2)*cos(q1);
      q3*cos(q2)*sin(q1);
      d1 + q3 * sin(q2)];
q = [q1; q2; q3];
q_d = [q1_d; q2_d; q3_d];
J = jacobian(fr, q);
display(J)
r_d = simplify(analityc_J(fr, q, q_d));
display(r_d)

% Example slide 11 pag 19
syms l1 l2 q1 q2 q1_d q2_d real
DH_table = [l1 0 0 q1;
            l2 0 0 q2];
joint_types = ['r' 'r'];
q = [q1; q2];
q_d = [q1_d; q2_d];
[v_E, w_E, J] = geometric_J(DH_table, joint_types, q_d);
v_E = simplify(v_E);
display(v_E)
w_E = simplify(w_E);
display(w_E)
J = simplify(J);
display(J)

