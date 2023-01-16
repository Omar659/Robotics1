clear all
clc

addpath("Functions\")

Rzxz = euler_RPY([0 0 1], [1 0 0], [0 0 1], sym("phi"), sym("theta"), sym("psi"), true);
disp(Rzxz)

v_p = [1 1 0]';

v = Rzxz * v_p;

v = subs(v, {'phi','theta','psi'}, {pi, -pi/2, 0});
disp(v)
Rzxz = subs(Rzxz, {'phi','theta','psi'}, {pi, -pi/2, 0});
theta = simplify(atan2(sqrt(Rzxz(3,1)^2 + Rzxz(3,2)^2), Rzxz(3,3)));
phi = simplify(atan2(Rzxz(1,3)/sin(theta), -Rzxz(2,3)/sin(theta)));
psi = simplify(atan2(Rzxz(3,1)/sin(theta), Rzxz(3,2)/sin(theta)));
display(theta)
display(phi)
display(psi)

R = euler_RPY([1 0 0], [0 1 0], [0 0 1], sym("psi"), sym("theta"), sym("phi"), false);
disp(R)
R = subs(R, {'phi','theta','psi'}, {-pi/2, pi, 0});
[sol1, sol2] = angles_from_RPY(R);
disp(sol1)
disp(sol2)

a_T_b = homogeneous_T(rotation_around_r([0 0 1], sym("theta")), [1;1;1]);
disp(a_T_b)
b_T_a = inverse_T(a_T_b);
disp(b_T_a)