clear all
clc

addpath("Functions\")

w_R_zxz = euler_RPY([0 0 1], [1 0 0], [0 0 1], sym("phi"), sym("theta"), sym("psi"), true);
disp("Rotation matrix with moving axis around Z -> X' -> Z''")
disp(w_R_zxz)

disp("Check if it works")
disp("Given this vector in the world reference frame:")
w_v = [1 1 0]';
disp(w_v)

disp("Compute the vector wrt the new reference frame ZX'Z''")
zxz_v = w_R_zxz * w_v;
disp(zxz_v)

disp("Substitute the angle theta = -pi/2; phi = pi; psi = 0")
zxz_v = subs(zxz_v, {'phi','theta','psi'}, {pi, -pi/2, 0});
disp("We expect as output the vector [-1; 0; -1]")
disp("The output is:")
disp(zxz_v)

disp("############################################")
disp("MOVING AXIS")
disp("We want to compute the three angle involved in the rotation.")
disp(w_R_zxz)

w_R_zxz = subs(w_R_zxz, {'phi','theta','psi'}, {pi, -pi/2, 0});
disp("First, substitute in the matrix, as example, the angles with theta = -pi/2; phi = pi; psi = 0")
disp(w_R_zxz)

disp("I can compute an angle with the atan2(sin(angle), cos(angle)). Just find this sin and cosine in the rotation matrix!")
theta1 = simplify(atan2(sqrt(w_R_zxz(3,1)^2 + w_R_zxz(3,2)^2), w_R_zxz(3,3)));
phi1 = simplify(atan2(w_R_zxz(1,3)/sin(theta1), -w_R_zxz(2,3)/sin(theta1)));
psi1 = simplify(atan2(w_R_zxz(3,1)/sin(theta1), w_R_zxz(3,2)/sin(theta1)));
theta2 = simplify(atan2(sqrt(w_R_zxz(3,1)^2 + w_R_zxz(3,2)^2), w_R_zxz(3,3)));
phi2 = simplify(atan2(w_R_zxz(1,3)/sin(theta2), -w_R_zxz(2,3)/sin(theta2)));
psi2 = simplify(atan2(w_R_zxz(3,1)/sin(theta2), w_R_zxz(3,2)/sin(theta2)));
disp("These are the angles. Actually two results:")
disp("With positive square root")
display(theta1)
display(phi1)
display(psi1)
disp("With negative square root")
display(theta2)
display(phi2)
display(psi2)

disp("FIXED AXIS")
R = euler_RPY([1 0 0], [0 1 0], [0 0 1], sym("psi"), sym("theta"), sym("phi"), false);
disp("We want to compute the three angle involved in the rotation.")
disp("Compute the Roll-Pitch-Yaw rotation matrix")
disp(R)

R = subs(R, {'phi','theta','psi'}, {-pi/2, pi, 0});
disp("First, substitute in the matrix, as example, the angles with theta = -pi/2; phi = pi; psi = 0")
disp(R)

disp("I can compute an angle with the atan2(sin(angle), cos(angle)). Just find this sin and cosine in the rotation matrix!")
disp("These are the angles. Actually two results.")
[sol1, sol2] = angles_from_RPY(R);
disp("With positive square root")
disp(sol1)
disp("With negative square root")
disp(sol2)

disp("############################################")
disp("HOMOGENEOUS TRANSFORMATION TESTS")

a_T_b = homogeneous_T(rotation_around_r([0 0 1], sym("theta")), [1;1;1]);
disp("A 4x4 transformation matrix:")
disp(a_T_b)

b_T_a = inverse_T(a_T_b);
disp("The inverse of the previous 4x4 transformation matrix:")
disp(b_T_a)