addpath("Functions/") % Linux
% addpath("Functions\") % Windows

%%
clear all
close all
clc

syms a b g ad bd gd  real
RPY_d = derivative_RPY(1, 3, 2, [a b g])

det_RPY_d = simplify(det(RPY_d))

RPY_d_s = simplify(subs(RPY_d, {b}, {pi/2}))

w = RPY_d_s*[ad; bd; gd]

null_RPY_d_s = simplify(null(RPY_d_s))

simplify(orth(RPY_d_s))


%%
clear all
close all
clc

syms L M N q1 q2 q3 px py pz real

p_x = L*cos(q1) + N*cos(q1 + q2)*cos(q3)
p_y = L*sin(q1) + N*sin(q1 + q2)*cos(q3)
p_z = M + N*sin(q3)

s3 = (pz - M)/N
c3_p = simplify(sqrt(1-s3^2))
c3_n = simplify(-c3_p)

q_3_p = simplify(atan2(s3, c3_p))
q_3_n = simplify(atan2(s3, c3_n))

c2 = (px^2 + py^2 - 2*(N^2)*cos(q3) - L^2)/(2*L*N*cos(q3))
s2_p = simplify(sqrt(1-c2^2))
s2_n = simplify(-s2_p)

q_2_p = simplify(atan2(s2_p, c2))
q_2_n = simplify(atan2(s2_n, c2))

syms s_1 c_1
A = [L + N*cos(q3)*cos(q2), -N*cos(q3)*sin(q2);
     N*cos(q3)*sin(q2), L + N*cos(q3)*cos(q2)]

x = simplify(inv(A)*[px; py]);

c1 = x(1)
s1 = x(2)

q_1 = simplify(atan2(s1, c1))

%%
fq3 = vpa(simplify(subs(q_3_p, {L, M, N, px, py, pz}, ...
        {0.5, 0.5, 0.5, 0.3, -0.3, 0.7})), 4)
fq2 = vpa(simplify(subs(q_2_p, {L, M, N, px, py, pz, q3}, ...
        {0.5, 0.5, 0.5, 0.3, -0.3, 0.7, fq3})), 4)
fq1 = vpa(simplify(subs(q_1, {L, M, N, px, py, pz, q3, q2}, ...
        {0.5, 0.5, 0.5, 0.3, -0.3, 0.7, fq3, fq2})), 4)
fpx = vpa(simplify(subs(p_x, {L, M, N, q1, q2, q3}, ...
    {0.5, 0.5, 0.5, fq1, fq2, fq3})), 4)
fpy = vpa(simplify(subs(p_y, {L, M, N, q1, q2, q3}, ...
    {0.5, 0.5, 0.5, fq1, fq2, fq3})), 4)
fpz = vpa(simplify(subs(p_z, {L, M, N, q1, q2, q3}, ...
    {0.5, 0.5, 0.5, fq1, fq2, fq3})), 4)

sq3 = vpa(simplify(subs(q_3_n, {L, M, N, px, py, pz}, ...
        {0.5, 0.5, 0.5, 0.3, -0.3, 0.7})), 4)
sq2 = vpa(simplify(subs(q_2_p, {L, M, N, px, py, pz, q3}, ...
        {0.5, 0.5, 0.5, 0.3, -0.3, 0.7, sq3})), 4)
sq1 = vpa(simplify(subs(q_1, {L, M, N, px, py, pz, q3, q2}, ...
        {0.5, 0.5, 0.5, 0.3, -0.3, 0.7, sq3, sq2})), 4)
spx = vpa(simplify(subs(p_x, {L, M, N, q1, q2, q3}, ...
    {0.5, 0.5, 0.5, sq1, sq2, sq3})), 4)
spy = vpa(simplify(subs(p_y, {L, M, N, q1, q2, q3}, ...
    {0.5, 0.5, 0.5, sq1, sq2, sq3})), 4)
spz = vpa(simplify(subs(p_z, {L, M, N, q1, q2, q3}, ...
    {0.5, 0.5, 0.5, sq1, sq2, sq3})), 4)

tq3 = vpa(simplify(subs(q_3_p, {L, M, N, px, py, pz}, ...
        {0.5, 0.5, 0.5, 0.3, -0.3, 0.7})), 4)
tq2 = vpa(simplify(subs(q_2_n, {L, M, N, px, py, pz, q3}, ...
        {0.5, 0.5, 0.5, 0.3, -0.3, 0.7, tq3})), 4)
tq1 = vpa(simplify(subs(q_1, {L, M, N, px, py, pz, q3, q2}, ...
        {0.5, 0.5, 0.5, 0.3, -0.3, 0.7, tq3, tq2})), 4)
tpx = vpa(simplify(subs(p_x, {L, M, N, q1, q2, q3}, ...
    {0.5, 0.5, 0.5, tq1, tq2, tq3})), 4)
tpy = vpa(simplify(subs(p_y, {L, M, N, q1, q2, q3}, ...
    {0.5, 0.5, 0.5, tq1, tq2, tq3})), 4)
tpz = vpa(simplify(subs(p_z, {L, M, N, q1, q2, q3}, ...
    {0.5, 0.5, 0.5, tq1, tq2, tq3})), 4)

qq3 = vpa(simplify(subs(q_3_n, {L, M, N, px, py, pz}, ...
        {0.5, 0.5, 0.5, 0.3, -0.3, 0.7})), 4)
qq2 = vpa(simplify(subs(q_2_n, {L, M, N, px, py, pz, q3}, ...
        {0.5, 0.5, 0.5, 0.3, -0.3, 0.7, qq3})), 4)
qq1 = vpa(simplify(subs(q_1, {L, M, N, px, py, pz, q3, q2}, ...
        {0.5, 0.5, 0.5, 0.3, -0.3, 0.7, qq3, qq2})), 4)
qpx = vpa(simplify(subs(p_x, {L, M, N, q1, q2, q3}, ...
    {0.5, 0.5, 0.5, qq1, qq2, qq3})), 4)
qpy = vpa(simplify(subs(p_y, {L, M, N, q1, q2, q3}, ...
    {0.5, 0.5, 0.5, qq1, qq2, qq3})), 4)
qpz = vpa(simplify(subs(p_z, {L, M, N, q1, q2, q3}, ...
    {0.5, 0.5, 0.5, qq1, qq2, qq3})), 4)


%%


clear all
close all
clc

syms q1 q2 q3 real

L = 0.5;
M = 0.5;
N = 0.5;

f_r = [L*cos(q1) + N*cos(q1 + q2)*cos(q3);
       L*sin(q1) + N*sin(q1 + q2)*cos(q3);
       M + N*sin(q3)];

q0 = [-pi/4; pi/4; pi/4]
p_d = [0.3; -0.3; 0.7]

error = p_d - f_r;
J = simplify(jacobian(f_r, [q1 q2 q3]))
J_inv = inv(J);
[q_h, e_h, J_h] = newton_method(f_r, q0, p_d, [q1 q2 q3], 100, 3, 5, 0, 5, -1)
% q_k = q0;
% disp("1")
% q_k
% error_k = vpa(subs(error, {q1 q2 q3}, {q_k(1), q_k(2), q_k(3)}), 4)
% vpa(norm(error_k), 4) <= 10^(-3)
% J_k = subs(J_inv, {q1 q2 q3}, {q_k(1), q_k(2), q_k(3)});
% JJ_k = subs(J, {q1 q2 q3}, {q_k(1), q_k(2), q_k(3)});
% det_J_k = vpa(det(JJ_k), 4)
% 
% disp("2")
% q_k = vpa(q_k + J_k*error_k, 4)
% error_k = vpa(subs(error, {q1 q2 q3}, {q_k(1), q_k(2), q_k(3)}), 4)
% vpa(norm(error_k), 4) <= 10^(-3)
% J_k = subs(J_inv, {q1 q2 q3}, {q_k(1), q_k(2), q_k(3)});
% JJ_k = subs(J, {q1 q2 q3}, {q_k(1), q_k(2), q_k(3)});
% det_J_k = vpa(det(JJ_k), 4)
% 
% disp("3")
% q_k = vpa(q_k + J_k*error_k, 4)
% error_k = vpa(subs(error, {q1 q2 q3}, {q_k(1), q_k(2), q_k(3)}), 4)
% vpa(norm(error_k), 4) <= 10^(-3)
% J_k = subs(J_inv, {q1 q2 q3}, {q_k(1), q_k(2), q_k(3)});
% JJ_k = subs(J, {q1 q2 q3}, {q_k(1), q_k(2), q_k(3)});
% det_J_k = vpa(det(JJ_k), 4)
% 
% disp("4")
% q_k = vpa(q_k + J_k*error_k, 4)
% error_k = vpa(subs(error, {q1 q2 q3}, {q_k(1), q_k(2), q_k(3)}), 4)
% vpa(norm(error_k), 4) <= 10^(-3)
% J_k = subs(J_inv, {q1 q2 q3}, {q_k(1), q_k(2), q_k(3)});
% det_J_k = vpa(det(JJ_k), 4)
% 
% disp("5")
% q_k = vpa(q_k + J_k*error_k, 4)
% error_k = vpa(subs(error, {q1 q2 q3}, {q_k(1), q_k(2), q_k(3)}), 4)
% vpa(norm(error_k), 4) <= 10^(-3)
% J_k = subs(J_inv, {q1 q2 q3}, {q_k(1), q_k(2), q_k(3)});
% JJ_k = subs(J, {q1 q2 q3}, {q_k(1), q_k(2), q_k(3)});
% det_J_k = vpa(det(JJ_k), 4)
% 
% disp("6")
% q_k = vpa(q_k + J_k*error_k, 4)
% error_k = vpa(subs(error, {q1 q2 q3}, {q_k(1), q_k(2), q_k(3)}), 4)
% vpa(norm(error_k), 4) <= 10^(-3)





%%
clear all
close all
clc


syms q1 q2 q3 L M N real

f_r = [L*cos(q1) + N*cos(q1 + q2)*cos(q3);
       L*sin(q1) + N*sin(q1 + q2)*cos(q3);
       M + N*sin(q3)];

J = simplify(jacobian(f_r, [q1 q2 q3]))
det_J = simplify(det(J))

inv_J = simplify(inv(J))

q_s = [-pi/4; pi/4; pi/4]
q_g = [0; 0; pi/4]

pd_s = [1; -1; 0]
pd_g = [0; 0; 0]
qd_s = subs(simplify(inv_J*pd_s), {q1, q2, q3}, {q_s(1), q_s(2), q_s(3)})
qd_g = subs(simplify(inv_J*pd_g), {q1, q2, q3}, {q_g(1), q_g(2), q_g(3)})

qd_s_lmn = subs(qd_s, {L, M, N}, {0.5, 0.5, 0.5})
qd_g_lmn = subs(qd_g, {L, M, N}, {0.5, 0.5, 0.5})








