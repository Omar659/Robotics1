clear all
close all
clc

% addpath("Functions/") % Linux
addpath("Functions\") % Windows

syms pi s s_d s_dd pf sigma L v_max a_max real

% s = sigma/L
ps = pi + s*(pf - pi);
% s_d = sigma_d/L
ps_d = (pf - pi) * s_d;
% s_dd = sigma_dd/L
ps_dd = (pf - pi) * s_dd;

[T_s,T] = bang_cost_bang(L, v_max, a_max)

ps = subs(ps, {s}, {sigma/L})
A = [3;3];
B = [1;9];
C = [8;9];
v1 = 1;
v2 = 2;

% d1 = 3;
% [d2,d_t] = overfly_dj_d_T(d1,v1,v2);

d_t = 0.4;
[d1,d2] = overfly_d1_d2(d_t, v1, v2);

t_ls = linspace(0, d_t, 100);
K_ab = (B-A)/norm(B-A);
K_bc = (C-B)/norm(C-B);
A_p = B - d1*K_ab;
p_t = A_p + v1 * K_ab*t_ls + ((v2*K_bc - v1*K_ab)*t_ls.^2)/ (2*d_t);
hold on
    text(A(1), A(2), 'A', Color='g')
    text(B(1), B(2), 'B', Color='r', HorizontalAlignment='right')
    text(C(1), C(2), 'C', Color='g')
    plot([A(1) B(1) C(1)], [A(2) B(2) C(2)])
    plot(p_t(1,:), p_t(2,:))
hold off
grid on
xlim([0 10])
ylim([0 10])