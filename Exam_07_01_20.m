clear all
close all
clc
% addpath("Functions/") % Linux
addpath("Functions\") % Windows

syms q1 q2 q3 q4 q5 q6 q7 q1_d q2_d q3_d q4_d q5_d q6_d q7_d real
% DH_table = [0 -pi/2 0 0;
%             0 pi/2 0.356 0;
%             0 -pi/2 -0.635 pi/2;
%             0.508 0 0 0;
%             0 pi/2 0 0;
%             0 pi/2 0 pi/2;
%             0 0 0.343 0]

DH_table = [0 -pi/2 0 q1;
            0 pi/2 0.356 q2;
            0 -pi/2 -0.635 q3;
            0.508 0 0 q4;
            0 pi/2 0 q5;
            0 pi/2 0 q6;
            0 0 0.343 q7]



A_01 = dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4));
A_12 = dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4));
A_23 = dh_matrix(DH_table(3,1), DH_table(3,2), DH_table(3,3), DH_table(3,4));
A_34 = dh_matrix(DH_table(4,1), DH_table(4,2), DH_table(4,3), DH_table(4,4));
A_45 = dh_matrix(DH_table(5,1), DH_table(5,2), DH_table(5,3), DH_table(5,4));
A_56 = dh_matrix(DH_table(6,1), DH_table(6,2), DH_table(6,3), DH_table(6,4));
A_67 = dh_matrix(DH_table(7,1), DH_table(7,2), DH_table(7,3), DH_table(7,4));

A_02 = A_01*A_12;
A_03 = A_01*A_12*A_23;
A_04 = A_01*A_12*A_23*A_34;
A_05 = A_01*A_12*A_23*A_34*A_45;
A_06 = A_01*A_12*A_23*A_34*A_45*A_56;
A_07 = A_01*A_12*A_23*A_34*A_45*A_56*A_67;

% O_A_i = [eye(4) A_01 A_02 A_03 A_04 A_05 A_06 A_07];

joint_types = ["r","r","r","r","r","r","r","ee"];

%plot_robot_pose_lite(joint_types,DH_table,O_A_i, false)
q_dot = [q1_d; q2_d; q3_d; q4_d; q5_d; q6_d; q7_d]
[v_E, w_E, J] = geometric_J(DH_table, joint_types, q_dot)
v_E
w_E
vpa(simplify(J),4)

A_04

%%
clear all
close all
clc

deg2rad(-60)
deg2rad(30)

R_0 = vpa(rotation_around_r([0 0 1], -1.0472),4)
R_T = vpa(rotation_around_r([0 0 1], 0.5236),4)

R_0T = vpa(R_0'*R_T,4)

theta = vpa(compute_theta(R_0T),4)

compute_r(R_0T, theta)

A = [5.2 1.5]
B = [2.2 -2.5]
norm(B-A)

%%
clear all
close all
clc

syms T t tau d_q real 
q_0 = [1;-0.5]
q_f = [0;0.2]

d_q1 = q_f(1) - q_0(1)
d_q2 = q_f(2) - q_0(2)

V1 = 0.5;
V2 = 0.8;
A1 = 0.8;
A2 = 0.5;

q_d_t = (d_q/T)*(30*(tau)^4 - 60*(tau)^3 + 30*(tau)^2)
q_dd_t = (d_q/T^2)*(120*(tau)^3 - 180*(tau)^2 + 60*(tau))
tau_0 = (9 - sqrt(65))/8
q1_d_t = (d_q1/T)*(30*(1/2)^4 - 60*(1/2)^3 + 30*(1/2)^2)
q1_dd_t = (d_q1/T^2)*(120*(tau_0)^3 - 180*(tau_0)^2 + 60*(tau_0))
q2_d_t = (d_q2/T)*(30*(1/2)^4 - 60*(1/2)^3 + 30*(1/2)^2)
q2_dd_t = (d_q2/T^2)*(120*(tau_0)^3 - 180*(tau_0)^2 + 60*(tau_0))

TminV1 = (abs(d_q1)/V1)*(30*(1/2)^4 - 60*(1/2)^3 + 30*(1/2)^2)
TminV2 = (abs(d_q2)/V2)*(30*(1/2)^4 - 60*(1/2)^3 + 30*(1/2)^2)
TminA1 = sqrt((abs(d_q1)/A1)*abs(120*(tau_0)^3 - 180*(tau_0)^2 + 60*(tau_0)))
TminA2 = sqrt((abs(d_q2)/A2)*abs(120*(tau_0)^3 - 180*(tau_0)^2 + 60*(tau_0)))

vpa(subs(q1_d_t,{T}, {TminA2}),4)
vpa(subs(q1_dd_t, {T}, {TminA2}),4)
vpa(subs(q2_d_t, {T}, {TminA2}),4)
vpa(subs(q2_dd_t, {T}, {TminA2}),4)

t_p = linspace(0,1,100);
a = subs(q_d_t,{d_q, tau, T},{d_q1,t_p, TminA2})
plot(t_p,a)
t_p = linspace(0,1,100);
a = subs(q_d_t,{d_q, tau, T},{d_q2,t_p, TminA2})
plot(t_p,a)
t_p = linspace(0,1,100);
a = subs(q_dd_t,{d_q, tau, T},{d_q1,t_p, TminA2})
plot(t_p,a)
t_p = linspace(0,1,100);
a = subs(q_dd_t,{d_q, tau, T},{d_q2,t_p, TminA2})
plot(t_p,a)

%%
clear all
close all
clc

syms  v1 v2 v3 v4 real
q_t1 = 45
q_t2 = 90
q_t3 = -45
q_t4 = 45
t1 = 1 -1
t2 = 2 -1
t3 = 2.5 -1
t4 = 4 -1
h1 = t2 - t1
h2 = t3 - t2
h3 = t4 - t3
% [th, thd, thdd] = compute_spline(2, [h1;h2;h3], [q_t1;q_t2;q_t3;q_t4], [0;v2;v3;0])
[th, thd, thdd] = compute_spline(3, [h1;h2;h3], [q_t1;q_t2;q_t3;q_t4], [0;v2;v3;0])

%%

t = linspace(0,1,100);
a = 3*t;
plot(t,a)
%%
clear all
close all
clc

syms dq tau T real
q1 = [rad2deg(pi/4);-100]
q2 = [rad2deg(-pi/2);100]
dq1 = q2(1) - q1(1) 
dq2 = q2(2) - q1(2)
q_d_t = (dq/T)*(30*(tau)^4 -60*(tau)^3 + 30*(tau)^2)
q_dd_t = (dq/T^2)*(120*(tau)^3 -180*(tau)^2 + 60*(tau))

tau_a = (3-sqrt(3))/6
vpa(subs(q_d_t, {dq,tau,T}, {dq1,0.5,2.7745}),4)
vpa(subs(q_dd_t, {dq,tau,T}, {dq1,tau_a,2.7745}),4)
vpa(subs(q_d_t, {dq,tau,T}, {dq2,0.5,2.7745}),4)
vpa(subs(q_dd_t, {dq,tau,T}, {dq2,tau_a,2.7745}),4)

(dq1/180)*(30*(0.5)^4 -60*(0.5)^3 + 30*(0.5)^2)
sqrt(abs((dq2/150))*abs((120*(tau_a)^3 -180*(tau_a)^2 + 60*(tau_a))))

t = linspace(0,1,100);
a = (dq/T)*(30*(tau)^4 -60*(tau)^3 + 30*(tau)^2)
a = subs(a, {dq, T, tau}, {dq1, 2.7745, t})
plot(t,a)