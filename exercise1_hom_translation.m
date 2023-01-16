clear all
clc

addpath("Functions\")

t_p = [sym("p_x"); sym("p_y"); 0];
disp(t_p)
e_R_t = rotation_around_r([1 0 0], pi);
disp(e_R_t)
e_p = [0; 0; sym("h")];
disp(e_p)
e_p_et = e_p - e_R_t * t_p;
e_T_t = homogeneous_T(e_R_t, e_p_et);
disp(e_T_t)
t_T_e = inverse_T(e_T_t);
disp(t_T_e)
t_p_te = t_T_e(1:3, 4);
disp(t_p_te)
% t_p = t_p_te + t_R_e * e_p;
% e_p = e_p_et + e_R_t * t_p;