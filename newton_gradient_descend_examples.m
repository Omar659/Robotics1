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
r_d = [1;1;1]; % desired EE position
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
newton_method(fr, q_0, r_d, q, 15, 5, 6, q_des, 4, fig_speed)
% Apply gradient descend method
figure(2)
gradient_descend(fr, q_0, r_d, q, 0.7, 15, 5, 6, q_des, fig_speed)