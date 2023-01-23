clear all
close all
clc

addpath("Functions/") % Linux
% addpath("Functions\") % Windows

disp("EXERCISE slide 8 pag 14")
disp("The robot carries a depth camera (e.g., a Kinect) on the end-effector")
disp("The end-effector should go to a pose above the point P on the table, ")
disp("pointing its approach axis z_e downward and being aligned with the table sides")
disp(" ")

e_R_t = rotation_around_r([1 0 0], pi);
disp("End effector RF pointing downward")
disp(e_R_t)

t_p = [sym("p_x"); sym("p_y"); 0];
disp("point P is known in the table frame RF_t")
disp(t_p)

e_p = [0; 0; sym("h")];
disp("The robot proceeds by centering point P in its camera image until")
disp("it senses a depth h from the table (in RF_e)")
disp(e_p)

% e_p = e_p_et + e_R_t * t_p;
e_p_et = e_p - e_R_t * t_p;
disp("Compute the vector from the origin of the end effector to the origin of the table")
disp(e_p_et)

e_T_t = homogeneous_T(e_R_t, e_p_et);
disp("Then compute the homogeneous transformation of the table wrt the end effector")
disp(e_T_t)

t_T_e = inverse_T(e_T_t);
disp("Compute the inverse of the the homogeneous transoformation of the table wrt the end effector")
disp("in order to have the homogenous transformation of the end effector wrt the table")
disp(t_T_e)

t_p_te = t_T_e(1:3, 4);
disp("Then the last row (first three element) is the position of the end effector wrt the table")
disp(t_p_te)
