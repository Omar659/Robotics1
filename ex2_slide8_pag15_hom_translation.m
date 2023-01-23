clear all
close all
clc

addpath("Functions/") % Linux
% addpath("Functions\") % Windows

q_a = [pi/3;-pi/2];
w_p_a = [-2.5; 1; 0];
w_p_b = [1; 2; 0];
alpha_b = pi/6;
link_len = 1; 

w_T_a = homogeneous_T(eye(3), w_p_a);
disp(w_T_a)

a_p_aea_x = cos(q_a(1)) * link_len + cos(q_a(1) + q_a(2)) * link_len;
a_p_aea_y = sin(q_a(1)) * link_len + sin(q_a(1) + q_a(2)) * link_len;
a_p_aea = [a_p_aea_x; a_p_aea_y; 0];
a_R_ea = rotation_around_r([0 0 1], q_a(1) + q_a(2));
a_T_ea = homogeneous_T(a_R_ea, a_p_aea);
disp(vpa(a_T_ea,4))

ea_R_eb = rotation_around_r([0 0 1], pi);
ea_p_eaeb = [0; 0; 0];
ea_T_eb = homogeneous_T(ea_R_eb, ea_p_eaeb);
disp(ea_T_eb)


w_R_b = rotation_around_r([0 0 1], alpha_b);
w_T_b = homogeneous_T(w_R_b, w_p_b);
disp(vpa(w_T_b,4))

syms q_b1 q_b2 q_b3 real
b_p_beb_x = cos(q_b1) * link_len + ...
            cos(q_b1 + q_b2) * link_len + ...
            cos(q_b1 + q_b2 + q_b3) * link_len;
b_p_beb_y = sin(q_b1) * link_len + ...
            sin(q_b1 + q_b2) * link_len + ...
            sin(q_b1 + q_b2 + q_b3) * link_len;
b_p_beb = [b_p_beb_x; b_p_beb_y; 0];
b_R_eb = rotation_around_r([0 0 1], q_b1 + q_b2 + q_b3);
b_T_eb = homogeneous_T(b_R_eb, b_p_beb);
disp(b_T_eb)

% w_T_a * a_T_ea * ea_T_eb = w_T_b * b_T_eb;
b_T_w = inverse_T(w_T_b);
b_T_eb = b_T_w * w_T_a * a_T_ea * ea_T_eb;

disp(vpa(b_T_eb,4))
% disp(compute_theta(b_T_eb(1:3, 1:3)))

