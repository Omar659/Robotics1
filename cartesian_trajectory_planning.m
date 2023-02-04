clear all
close all
clc

% addpath("Functions/") % Linux
addpath("Functions\") % Windows

syms p_i s s_d s_dd pf sigma L v_max a_max real

% s = sigma/L
ps = p_i + s*(pf - p_i);
% s_d = sigma_d/L
ps_d = (pf - p_i) * s_d;
% s_dd = sigma_dd/L
ps_dd = (pf - p_i) * s_dd;

[T_s,T] = bang_cost_bang(L, v_max, a_max)

ps = subs(ps, {s}, {sigma/L})

% Example slide 14 pag 10
% From
A = [3;3];
% To
C = [8;9];
% Via
B = [1;9];
% Starting velocity of the transition (overfly)
v1 = 1;
% Ending velocity of the transition (overfly)
v2 = 2;

% Case 1: given d1
% d1 = 3;
% [d2,d_t] = overfly_dj_d_T(d1,v1,v2);

% Case 2: given d_t
d_t = 4;
[d1,d2] = overfly_d1_d2(d_t, v1, v2);

% Interpolation of the path
t_ls = linspace(0, d_t, 100);
% Unit vector from A to B
K_ab = (B-A)/norm(B-A);
% Unit vector from B to C
K_bc = (C-B)/norm(C-B);
% Starting point position of the transition
A_p = B - d1*K_ab;
% Transition path formula
p_t = A_p + v1 * K_ab*t_ls + ((v2*K_bc - v1*K_ab)*t_ls.^2)/ (2*d_t);
% Plot
figure(1)
hold on
    text(A(1), A(2), 'A', Color='g')
    text(B(1), B(2), 'B', Color='r', HorizontalAlignment='right')
    text(C(1), C(2), 'C', Color='g')
    line([A(1) B(1) C(1)], [A(2) B(2) C(2)])
    line(p_t(1,:), p_t(2,:))
hold off
grid on
xlim([0 10])
ylim([0 10])

syms r h s real
% Viviani curve
s_ls = linspace(-pi/2, pi/2, 100);
x = r*cos(s)^2;
y = r*cos(s)*sin(s);
z = r*sin(s);
x_plot = subs(x, {s, r}, {s_ls, 1});
y_plot = subs(y, {s, r}, {s_ls, 1});
z_plot = subs(z, {s, r}, {s_ls, 1});
[t_s, n_s, b_s, k_s, tau_s] = frenet_frame([x; y; z], s)
figure(2)
% for i = 1:100
    clf
    view(3)
    i = 50;
    t_s_plot = subs(t_s, {s, r}, {s_ls(i), 1});
    n_s_plot = subs(n_s, {s, r}, {s_ls(i), 1});
    b_s_plot = subs(b_s, {s, r}, {s_ls(i), 1});
    point = [subs(x, {s, r}, {s_ls(i), 1});
             subs(y, {s, r}, {s_ls(i), 1});
             subs(z, {s, r}, {s_ls(i), 1})];
    hold on
        line(x_plot, y_plot, z_plot)
        quiver3(point(1), point(2), point(3), t_s_plot(1), t_s_plot(2), t_s_plot(3))
        text(point(1) + t_s_plot(1), point(2) + t_s_plot(2), point(3) + t_s_plot(3), "t(s)")
        quiver3(point(1), point(2), point(3), n_s_plot(1), n_s_plot(2), n_s_plot(3))
        text(point(1) + n_s_plot(1), point(2) + n_s_plot(2), point(3) + n_s_plot(3), "n(s)")
        quiver3(point(1), point(2), point(3), b_s_plot(1), b_s_plot(2), b_s_plot(3))
        text(point(1) + b_s_plot(1), point(2) + b_s_plot(2), point(3) + b_s_plot(3), "b(s)")
    hold off
    grid on
    xlim([double(min([x_plot y_plot z_plot]))-1 double(max([x_plot y_plot z_plot]))+1])
    ylim([double(min([x_plot y_plot z_plot]))-1 double(max([x_plot y_plot z_plot]))+1])
    zlim([double(min([x_plot y_plot z_plot]))-1 double(max([x_plot y_plot z_plot]))+1])
%     pause(0.005)
% end

% Helix curve
s_ls = linspace(0, 2*pi, 100);
x = r*cos(s);
y = r*sin(s);
z = h*s;
x_plot = subs(x, {s, r, h}, {s_ls, 1, 2});
y_plot = subs(y, {s, r, h}, {s_ls, 1, 2});
z_plot = subs(z, {s, r, h}, {s_ls, 1, 2});
[t_s, n_s, b_s, k_s, tau_s] = frenet_frame([x; y; z], s)
figure(3)
% for i = 1:100
    clf
    view(3)
    i = 50;
    t_s_plot = subs(t_s, {s, r, h}, {s_ls(i), 1, 2});
    n_s_plot = subs(n_s, {s, r, h}, {s_ls(i), 1, 2});
    b_s_plot = subs(b_s, {s, r, h}, {s_ls(i), 1, 2});
    point = [subs(x, {s, r, h}, {s_ls(i), 1, 2});
             subs(y, {s, r, h}, {s_ls(i), 1, 2});
             subs(z, {s, r, h}, {s_ls(i), 1, 2})];
    hold on
        line(x_plot, y_plot, z_plot)
        quiver3(point(1), point(2), point(3), t_s_plot(1), t_s_plot(2), t_s_plot(3))
        text(point(1) + t_s_plot(1), point(2) + t_s_plot(2), point(3) + t_s_plot(3), "t(s)")
        quiver3(point(1), point(2), point(3), n_s_plot(1), n_s_plot(2), n_s_plot(3))
        text(point(1) + n_s_plot(1), point(2) + n_s_plot(2), point(3) + n_s_plot(3), "n(s)")
        quiver3(point(1), point(2), point(3), b_s_plot(1), b_s_plot(2), b_s_plot(3))
        text(point(1) + b_s_plot(1), point(2) + b_s_plot(2), point(3) + b_s_plot(3), "b(s)")
    hold off
    grid on
    xlim([double(min([x_plot y_plot z_plot]))-1 double(max([x_plot y_plot z_plot]))+1])
    ylim([double(min([x_plot y_plot z_plot]))-1 double(max([x_plot y_plot z_plot]))+1])
    zlim([double(min([x_plot y_plot z_plot]))-1 double(max([x_plot y_plot z_plot]))+1])
%     pause(0.005)
% end

