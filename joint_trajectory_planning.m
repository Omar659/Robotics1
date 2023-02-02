clear all
close all
clc

% addpath("Functions/") % Linux
addpath("Functions\") % Windows

syms a b c d e f t q0 q1 v0 v1 a0 a1 T real 
q_t = a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f 
q_t_p = diff(q_t, t)
q_t_s = diff(q_t_p, t)


f_v = q0
e_v = v0*T
d_v = (a0*T^2)/2
q_t1 = subs(q_t, {t, d, e, f}, {1, d_v, e_v, f_v}) - q1
q_t_p1 = subs(q_t_p, {t, d, e, f}, {1, d_v, e_v, f_v}) - v1*T
q_t_s1 = subs(q_t_s, {t, d, e, f}, {1, d_v, e_v, f_v}) - a1*T^2

[A, B] = equationsToMatrix([q_t1, q_t_p1, q_t_s1], [a, b, c]);
SOL = linsolve(A,B);
a_v = simplify(SOL(1))
b_v = simplify(SOL(2))
c_v = simplify(SOL(3))

q_t = simplify(subs(q_t, {a, b, c, d, e, f}, {a_v, b_v, c_v, d_v, e_v, f_v}))
q_t = simplify(subs(q_t, {q0, q1, v0, v1, a0, a1}, {0, 1, 0, 0, 0, 0}))

x = [0 1 2.5 3.6 5 7 8.1 10];
y = sin(x);
xx = 0:.25:10;
yy = spline(x,y,xx);
plot(x,y,'o',xx,yy)