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
T_RPY = simplify(derivative_RPY([alp bet gam]));
display(T_RPY)
