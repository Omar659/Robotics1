clear all
close all
clc

addpath("Functions/") % Linux
% addpath("Functions\") % Windows


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

% Test ex. Jai
A_01 = dh_matrix(sym("a1"), -pi/2, sym("a2"), pi/2);
A_12 = dh_matrix(sym("a2"), 0, 0, pi/2);
A_23 = dh_matrix(sym("a3"), 0, 0, -pi/2);
A_03 = A_01*A_12*A_23;
disp(A_03*[0;0;0;1])

% Test for nsap in the slide
A_01 = dh_matrix(0, -pi/2, 0, sym("theta1"));
A_12 = dh_matrix(sym("a2"), 0, 0, sym("theta2"));
A_23 = dh_matrix(sym("a3"), pi/2, sym("d3"), sym("theta3"));
A_34 = dh_matrix(0, -pi/2, sym("d4"), sym("theta4"));
A_45 = dh_matrix(0, pi/2, 0, sym("theta5"));
A_56 = dh_matrix(0, 0, 0, sym("theta6"));

A_06 = A_01*A_12*A_23*A_34*A_45*A_56;

n = simplify(A_06(1:3, 1));
s = simplify(A_06(1:3, 2));
a = simplify(A_06(1:3, 3));
p = simplify(A_06(1:3, 4));

display(n)
display(s)
display(a)
display(p)

% % data
% joint_range=700 %[deg] % range of the flange rotation
% nr=30 % reduction ratio
% res_joint=0.02 %[deg] % desired resolution at the flange side
% % computation
% disp('all angles are in degrees')
% turns_joint=joint_range/360
% turns_motor=nr*turns_joint
% bits_turn=ceil(log2(turns_motor))-1
% sectors_joint=360/res_joint
% tracks_motor=sectors_joint/nr
% % res_motor=tracks_motor/360
% bits_res=ceil(log2(tracks_motor))
% bits=bits_turn+bits_res
% % end
