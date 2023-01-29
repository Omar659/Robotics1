clear all
close all
clc

% addpath("Functions/") % Linux
addpath("Functions\") % Windows

syms q1 q2 q3 px py pz real

d1 = 0.5;
% Obtain EE catesian coordinates in function of q
fr = [q3 * cos(q2)* cos(q1); %px
      q3 * cos(q2)* sin(q1); %py
      d1 + q3 * sin(q2)];    %pz

q_0 = [0;0;1]; % initial configuration
r_d = [-1;-1;0]; % desired EE position
q = [q1; q2; q3]; 
% q function of p
% Obtain q3 in function of px, py, pz and d1
q_3 = sqrt(px^2 + py^2 + (pz - d1)^2);
% Obtain the two solution of q2
q_2_p = atan2((pz-d1)/q_3, sqrt(px^2 + py^2)/q_3);
q_2_s = atan2((pz-d1)/q_3, -sqrt(px^2 + py^2)/q_3);
% Obtain the two solution of q1 
q_1_p = atan2(py/cos(q_2_p), px/cos(q_2_p));
q_1_s = atan2(py/cos(q_2_s), px/cos(q_2_s));
% obtain the two desired configurations 
q_des = simplify([q_1_p q_1_s;
                  q_2_p q_2_s;
                  q_3 q_3]);
q_des = vpa(subs(q_des, {px, py, pz}, {1, 1, 1}), 5);
display(q_des)

% Animation speed
fig_speed = 0.1;
% Apply newton method
figure(1)
q_history_N = newton_method(fr, q_0, r_d, q, 15, 5, 6, q_des, 4, fig_speed);
% Apply gradient descend method
figure(2)
q_history_GD = gradient_descend(fr, q_0, r_d, q, 0.7, 150, 5, 6, q_des, fig_speed);

% figure(3)
% robot_animation(q_history_N, joint_positions, r_d, q, fig_speed*10)

figure(4);
clf
daspect([1 1 1])
q_0 = [q_1_p; q_2_p; q_3];
q_0 = vpa(simplify(subs(q_0, {px, py, pz}, {-1, -1, 0})), 4);
DH_table = [0.5, -pi/2, d1, q1;
            0.5, -pi/2, 0,  q2;
            0.5,  0,    q3, 0];
joint_types = ["r" "r" "p" "ee"];

A_01 = dh_matrix(0.5, -pi/2, d1, q1);
A_12 = dh_matrix(0.5, -pi/2, 0, q2);
A_23 = dh_matrix(0.5, 0, q3, 0);
A_02 = A_01*A_12;
A_03 = A_01*A_12*A_23;
O_A_i = [eye(4) A_01 A_02 A_03];
p = A_03(1:3, 4);
q_a = [q_0(1); q_0(2)-pi/2; q_0(3)];
O_A_i = vpa(simplify(subs(O_A_i, {q1, q2, q3}, {q_a(1), q_a(2), q_a(3)})) ,4);
DH_table = vpa(simplify(subs(DH_table, {q1, q2, q3}, {q_a(1), q_a(2), q_a(3)})) ,4);
plot_robot_pose(joint_types, DH_table, O_A_i)