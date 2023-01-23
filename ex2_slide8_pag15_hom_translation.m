clear all
close all
clc

addpath("Functions/") % Linux
% addpath("Functions\") % Windows

% Given data
% Joint angle robot A
q_a = [pi/3;-pi/2];
% Origin of the frame A
w_p_a = [-2.5; 1; 0];
% Origin of the frame of robot B
w_p_b = [1; 2; 0];
% Inclination angle around z-axis of the frame of Robot B
alpha_b = pi/6;
% All link length
link_len = 1; 

% Homogeneous transformation of the frame A wrt frame World
w_T_a = homogeneous_T(eye(3), w_p_a);
disp(w_T_a)
% Origin of the EE's frame of robot A
a_p_aea_x = cos(q_a(1)) * link_len + cos(q_a(1) + q_a(2)) * link_len;
a_p_aea_y = sin(q_a(1)) * link_len + sin(q_a(1) + q_a(2)) * link_len;
a_p_aea = [a_p_aea_x; a_p_aea_y; 0];
% Rotation of the EE's frame of Robot A
a_R_ea = rotation_around_r([0 0 1], q_a(1) + q_a(2));
% Homogeneous transformation of the EE of robot A
a_T_ea = homogeneous_T(a_R_ea, a_p_aea);
disp(vpa(a_T_ea,4))
% Rotation of the robot B EE wrt robot A EE frames
ea_R_eb = rotation_around_r([0 0 1], pi);
% Origin of the robot B EE wrt robot A EE frame. It is 0 because they are
% in the same point
ea_p_eaeb = [0; 0; 0];
% Homogeneous transformation of the EE of robot B wrt robot A
ea_T_eb = homogeneous_T(ea_R_eb, ea_p_eaeb);
disp(ea_T_eb)

% Rotation matrix of the frame B wrt frame World
w_R_b = rotation_around_r([0 0 1], alpha_b);
% Homogeneous transformation of the frame B wrt frame World
w_T_b = homogeneous_T(w_R_b, w_p_b);
disp(vpa(w_T_b,4))

syms q_b1 q_b2 q_b3 real
% Origin of the EE's frame of robot B
b_p_beb_x = cos(q_b1) * link_len + ...
            cos(q_b1 + q_b2) * link_len + ...
            cos(q_b1 + q_b2 + q_b3) * link_len;
b_p_beb_y = sin(q_b1) * link_len + ...
            sin(q_b1 + q_b2) * link_len + ...
            sin(q_b1 + q_b2 + q_b3) * link_len;
b_p_beb = [b_p_beb_x; b_p_beb_y; 0];
% Rotation of the EE's frame of Robot B
b_R_eb = rotation_around_r([0 0 1], q_b1 + q_b2 + q_b3);
% Homogeneous transformation of the EE of robot B
b_T_eb = homogeneous_T(b_R_eb, b_p_beb);
disp(b_T_eb)

% We have: w_T_a * a_T_ea * ea_T_eb = w_T_b * b_T_eb. Then:
b_T_w = inverse_T(w_T_b);
b_T_eb = b_T_w * w_T_a * a_T_ea * ea_T_eb;
disp(vpa(b_T_eb,4))