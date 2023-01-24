clear all
close all
clc

addpath("Functions\")
syms q1 q2 q3 px py pz real

d1 = 0.5;
fr = [q3 * cos(q2)* cos(q1);
      q3 * cos(q2)* sin(q1);
      d1 + q3 * sin(q2)];

q_0 = [0;0;1];
r_d = [1;1;1];
q = [q1; q2; q3];
q_3 = sqrt(px^2 + py^2 + (pz - d1)^2);
q_2_p = atan2((pz-d1)/q_3, sqrt(px^2 + py^2)/q_3);
q_2_s = atan2((pz-d1)/q_3, -sqrt(px^2 + py^2)/q_3);
q_1_p = atan2(py/cos(q_2_p), px/cos(q_2_p));
q_1_s = atan2(py/cos(q_2_s), px/cos(q_2_s));
q_des = simplify([q_1_p q_1_s;
                  q_2_p q_2_s;
                  q_3 q_3]);
q_des = vpa(subs(q_des, {px, py, pz}, {1, 1, 1}), 5);
display(q_des)

fig_speed = 0.3;
figure(1)
newton_method(fr, q_0, r_d, q, 15, 5, 6, q_des, 4, fig_speed)
figure(2)
gradient_descend(fr, q_0, r_d, q, 0.7, 15, 5, 6, q_des, fig_speed)