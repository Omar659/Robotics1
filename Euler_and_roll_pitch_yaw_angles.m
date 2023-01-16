clear all
clc

addpath("Functions\")

roll = rotation_around_r([1 0 0], sym('theta'));
disp(roll)
pitch = rotation_around_r([0 1 0], sym('phi'));
disp(pitch)
yaw = rotation_around_r([0 0 1], sym('psi'));
disp(yaw)

Rz = subs(yaw, {'psi'}, {sym('phi')});
Rx_p = roll;
Rz_s = yaw;

Rzxz = Rz * Rx_p * Rz_s;

v_p = [1 1 0]';

v = Rzxz * v_p;

v = subs(v, {'phi','theta','psi'}, {pi, -pi/2, 0});
Rzxz = subs(Rzxz, {'phi','theta','psi'}, {pi, -pi/2, 0});
theta = simplify(atan2(-sqrt(Rzxz(3,1)^2 + Rzxz(3,2)^2), Rzxz(3,3)));
phi = simplify(atan2(Rzxz(1,3)/sin(theta), -Rzxz(2,3)/sin(theta)));
psi = simplify(atan2(Rzxz(3,1)/sin(theta), Rzxz(3,2)/sin(theta)));
display(theta)
display(phi)
display(psi)

yaw = subs(yaw, {'psi'}, {sym('phi')});
pitch = subs(pitch, {'phi'}, {sym('theta')});
roll = subs(roll, {'theta'}, {sym('psi')});
Rrpy = yaw * pitch * roll;
disp(Rrpy)

Rrpy = subs(Rrpy, {'phi','theta','psi'}, {-pi/2, pi, 0});

theta = atan2(-Rrpy(3,1), -sqrt(Rrpy(3,2)^2 + Rrpy(3,3)^2));
phi = atan2(Rrpy(2,1)/cos(theta), Rrpy(1,1)/cos(theta));
psi = atan2(Rrpy(3,2)/cos(theta), Rrpy(3,3)/cos(theta));

disp(theta)
disp(phi)
disp(psi)