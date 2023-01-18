clear all
clc

addpath("Functions\")


A_01 = dh_matrix(0, sym('a1'), sym('d1'), sym('q1'));
A_12 = dh_matrix(0, sym('a2'), 0, sym('q2'));
A_23 = dh_matrix(0, 0, sym('q3'), 0);
A_34 = dh_matrix(pi, 0, sym('d4'), sym('q4'));

A_04 = A_01 * A_12 * A_23 * A_34;
disp(simplify(A_04))


A_01 = dh_matrix(0, pi/2, 0, sym('q1'));
A_12 = dh_matrix(0, -pi/2, 0, sym('q2'));
A_23 = dh_matrix(0, -pi/2, sym('L'), sym('q3'));
A_34 = dh_matrix(0, pi/2, 0, sym('q4'));
A_45 = dh_matrix(0, pi/2, sym('M'), sym('q5'));
A_56 = dh_matrix(0, -pi/2, 0, sym('q6'));
A_67 = dh_matrix(0, 0, 0, sym('q7'));

A_07 = A_01 * A_12 * A_23 * A_34 * A_45 * A_56 * A_67;
disp('Matrix A_01')
disp(A_01)
disp('Matrix A_12')
disp(A_12)
disp('Matrix A_23')
disp(A_23)
disp('Matrix A_34')
disp(A_34)
disp('Matrix A_45')
disp(A_45)
disp('Matrix A_56')
disp(A_56)
disp('Matrix A_67')
disp(A_67)

A_07 = subs(A_07,{'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'M', 'L'}, {0, pi/2, pi/2, -pi/2, 0, pi/2, 0, 0.39, 0.40});
% disp(vpa(A_07 *[0; 0.05; 0.1; 1]))

disp(A_07)
[SOL1, SOL2] = angles_from_RPY(A_07(1:3, 1:3));
disp(SOL1)
disp(SOL2)
