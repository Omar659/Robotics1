% addpath("Functions/") % Linux
addpath("Functions\") % Windows

%%
clear all
close all
clc

A = 2;
B = 1;
C = 3;
D = 0.7;
q1 = pi/2;
q2 = pi/4;
q3 = pi/6;
q4 = 3;

DH_table = [B,  pi/2, A, q1;
            C,  0,    0, q2;
            D,  pi/2, 0, q3;
            0,  0,   q4, 0];
joint_types = ["r" "r" "r" "p" "ee"];

A_01 = dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4));
A_12 = dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4));
A_23 = dh_matrix(DH_table(3,1), DH_table(3,2), DH_table(3,3), DH_table(3,4));
A_34 = dh_matrix(DH_table(4,1), DH_table(4,2), DH_table(4,3), DH_table(4,4));
A_02 = A_01*A_12;
A_03 = A_01*A_12*A_23;
A_04 = A_01*A_12*A_23*A_34;
O_A_i = [eye(4) A_01 A_02 A_03 A_04];
O_A_i = vpa(O_A_i,4);
DH_table = vpa(DH_table,4);
figure(1);
clf
daspect([1 1 1])
plot_robot_pose_lite(joint_types, DH_table, O_A_i, false)

%%
clear all
close all
clc

syms A B C D q1 q2 q3 q4 real
DH_table = [B,  pi/2, A, q1;
            C,  0,    0, q2;
            D,  pi/2, 0, q3;
            0,  0,   q4, 0];

A_01 = dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4));
A_12 = dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4));
A_23 = dh_matrix(DH_table(3,1), DH_table(3,2), DH_table(3,3), DH_table(3,4));
A_34 = dh_matrix(DH_table(4,1), DH_table(4,2), DH_table(4,3), DH_table(4,4));
A_04 = A_01*A_12*A_23*A_34;
f_r = A_04*[0;0;0;1];
f_r = simplify(f_r(1:3))
J = simplify(jacobian(f_r, [q1 q2 q3 q4]))
J_red = simplify(subs(J, {B, D}, {0, 0}))
det_J_red1 = simplify(det(J_red(:, 1:3)))
det_J_red2 = simplify(det([J_red(:, 1) J_red(:, 3:4)]))
det_J_red3 = simplify(det([J_red(:, 1:2) J_red(:, 4)]))
det_J_red4 = simplify(det(J_red(:, 2:4)))
J_red_s = simplify(subs(J_red, {q3, q4}, {0, 0}))

rank_J_red_s = rank(J_red_s)

range_J_red_s = [J_red_s(:, 1) J_red_s(:, 4)]

qd_star = [1; 0; 0; 1]

vd_s = J_red_s*qd_star

inv_Jrs = simplify(pinv(J_red_s))

vd_s = range_J_red_s(:, 2)

qd_star = simplify(inv_Jrs*vd_s)

vd_s = simplify(J_red_s*qd_star)

%%

%%
clear all
close all
clc



syms al be ga real
simplify(euler_RPY([0 0 1], [0 1 0], [1 0 0], al, be, ga, true));
RPY = simplify(euler_RPY([1 0 0], [0 1 0], [0 0 1], ga, be, al, false));
R1 = [0 -sqrt(2)/2 sqrt(2)/2;
      1 0 0;
      0 sqrt(2)/2 sqrt(2)/2]
R2 = [sqrt(2)/2 1/2 -1/2;
      0 -sqrt(2)/2 -sqrt(2)/2;
      -sqrt(2)/2 1/2 -1/2]
Rvia = [sqrt(6)/4 sqrt(2)/4 -sqrt(2)/2;
        -sqrt(6)/4 -sqrt(2)/4 -sqrt(2)/2;
        -1/2 sqrt(3)/2 0]

[ang1, sol1] = angles_from_RPY(R1);
[ang2, sol2] = angles_from_RPY(R2);
[ang_via, sol_via] = angles_from_RPY(Rvia);
ang1 = [ang1(3); ang1(2); ang1(1)]
ang2 = [ang2(3); ang2(2); ang2(1)]
ang_via = [ang_via(3); ang_via(2); ang_via(1)]

syms t v real

T1 = 2.5;
T2 = 1;
T = T1+T2;
d_1v = ang_via-ang1;
d_v2 = ang2-ang_via;

syms v
cA = 3 - (T1/d_1v)*v;
dA = (T1/d_1v)*v - 2;
cB = 3 + ((2*T2)/(T1*d_v2))*v;

% v_via = (6*T2^2*d_1v + 6*T1^4*d_v2)/(4*T1*T2^2 - 4*T2*T1^3);
% v_via = (3/(2*(T1+T2))) * ((T2/T1)*d_1v - (T1/T2)*d_v2);
x = (d_1v/T1^2)*(2*cA + 6*dA) - ((d_v2*T1^2)/T2^2)*2*cB;
v_via = [solve(x(1), v);
         solve(x(2), v);
         solve(x(3), v)];
v_via = vpa(v_via, 4);

tauA = t/T1;
aA = 0;
bA = 0;
cA = 3 - (T1/d_1v)*v_via;
dA = (T1/d_1v)*v_via - 2;
P_AN = aA + bA*tauA + cA*tauA^2 + dA*tauA^3;
P_A = vpa(ang1 + d_1v*P_AN, 4)

P_ANp = bA + 2*cA*tauA + 3*dA*tauA^2;
P_Ad = vpa((d_1v/T1)*P_ANp, 4)

P_ANpp = 2*cA + 6*dA*tauA;
P_Add = vpa((d_1v/(T1^2))*P_ANpp, 4)

tauB = (t-T1)/T2;
aB = 0;
bB = (T2/(T1*d_v2)) * v_via;
cB = 3 + ((2*T2)/(T1*d_v2))*v_via;
dB = -2 - (T2/(T1*d_v2))*v_via;
P_BN = aB + bB*tauB + cB*tauB^2 + dB*tauB^3;
P_B = vpa(ang_via + d_v2*P_BN, 4)

P_BNp = bB + 2*cB*tauB + 3*dB*tauB^2;
P_Bd = vpa((T1/T2)*d_v2*P_BNp, 4)

P_BNpp = 2*cB + 6*dB*tauB;
P_Bdd = vpa(((T1^2)/(T2^2))*d_v2*P_BNpp, 4)

xA = linspace(0, T1, 100);
xB = linspace(T1, T, 100);
P_A = subs(P_A, {t}, {xA});
% P_A = (207*pi)/112 * (t/T1)^3 -(291*pi)/112 * (t/T1)^2 + pi/2
% P_B = -(101*pi)/280 * ((t-T1)/T2 - 1)^3 -(171*pi)/280 * ((t-T1)/T2 - 1)^2
% P_A = subs(P_A, {t}, {xA});
% P_B = subs(P_B, {t}, {xB});
P_B = subs(P_B, {t}, {xB});
per_plot = [P_A P_B];

% figure(1)
% view(3)
% hold on
%     plot3(per_plot(1, :), per_plot(2, :), per_plot(3, :))
%     plot3(P_A(1, :), P_A(2, :), P_A(3, :))
%     plot3(P_B(1, :), P_B(2, :), P_B(3, :))
% hold off
% grid on
% 
figure(2)
hold on
%     plot([xA xB], per_plot(1, :))
    plot(xA, P_A(1, :))
    plot(xB, P_B(1, :))
hold off
grid on

figure(3)
hold on
%     plot([xA xB], per_plot(2, :))
    plot(xA, P_A(2, :))
    plot(xB, P_B(2, :))
hold off
grid on

figure(4)
hold on
%     plot([xA xB], per_plot(3, :))
    plot(xA, P_A(3, :))
    plot(xB, P_B(3, :))
hold off
grid on




%%
clear all
close all
clc
T1 = 2.5;
T2 = 1;
T = T1+T2;
R1 = [0 -sqrt(2)/2 sqrt(2)/2;
      1 0 0;
      0 sqrt(2)/2 sqrt(2)/2];
R2 = [sqrt(2)/2 1/2 -1/2;
      0 -sqrt(2)/2 -sqrt(2)/2;
      -sqrt(2)/2 1/2 -1/2];
Rvia = [sqrt(6)/4 sqrt(2)/4 -sqrt(2)/2;
        -sqrt(6)/4 -sqrt(2)/4 -sqrt(2)/2;
        -1/2 sqrt(3)/2 0];

[ang1, sol1] = angles_from_RPY(R1);
[ang2, sol2] = angles_from_RPY(R2);
[ang_via, sol_via] = angles_from_RPY(Rvia);
ang1 = [ang1(3); ang1(2); ang1(1)];
ang2 = [ang2(3); ang2(2); ang2(1)];
ang_via = [ang_via(3); ang_via(2); ang_via(1)];


% compute_spline(5, [sym("h1"); sym("h2"); sym("h3"); sym("h4"); sym("h5")], [sym("q1"); sym("q2"); sym("q3"); sym("q4"); sym("q5"); sym("q6")], [sym("v1"), sym("v2"), sym("v3"), sym("v4"), sym("v5"), sym("v6")])
% compute_spline(4, [sym("h1"); sym("h2"); sym("h3"); sym("h4")], [sym("q1"); sym("q2"); sym("q3"); sym("q4"); sym("q5")], [sym("v1"), sym("v2"), sym("v3"), sym("v4"), sym("v5")])
% compute_spline(3, [sym("h1"); sym("h2"); sym("h3")], [sym("q1"); sym("q2"); sym("q3"); sym("q4")], [sym("v1"), sym("v2"), sym("v3"), sym("v4")])
% compute_spline(2, [sym("h1"); sym("h2")], [sym("q1"); sym("q2"); sym("q3")], [sym("v1"), sym("v2"), sym("v3")])
% compute_spline(2, [sym("T1"); sym("T2")], [sym("q1"); sym("q2"); sym("q3")], [0, sym("v2"), 0])
[th1, thd1, thdd1] = compute_spline(2, [T1; T2], [ang1(1); ang_via(1); ang2(1)], [0, sym("v2"), 0]);
[th2, thd2, thdd2] = compute_spline(2, [T1; T2], [ang1(2); ang_via(2); ang2(2)], [0, sym("v2"), 0]);
[th3, thd3, thdd3] = compute_spline(2, [T1; T2], [ang1(3); ang_via(3); ang2(3)], [0, sym("v2"), 0]);

syms t real
th1 = vpa(th1, 4);
th2 = vpa(th2, 4);
th3 = vpa(th3, 4);
thd1 = vpa(thd1, 4);
thd2 = vpa(thd2, 4);
thd3 = vpa(thd3, 4);
thdd1 = vpa(thdd1, 4);
thdd2 = vpa(thdd2, 4);
thdd3 = vpa(thdd3, 4);

xA = linspace(0, T1, 100);
xB = linspace(T1, T, 100);

P_A = subs(thdd1(1), {t}, {xA});
P_B = subs(thdd1(2), {t}, {xB});

figure(1)
hold on
    plot(xA, P_A)
    plot(xB, P_B)
hold off
grid on

P_A = subs(thdd2(1), {t}, {xA});
P_B = subs(thdd2(2), {t}, {xB});

figure(2)
hold on
    plot(xA, P_A)
    plot(xB, P_B)
hold off
grid on

P_A = subs(thdd3(1), {t}, {xA});
P_B = subs(thdd3(2), {t}, {xB});

figure(3)
hold on
    plot(xA, P_A)
    plot(xB, P_B)
hold off
grid on



%%
clear all
close all
clc

syms r s h t real
fr = [r*cos(s);r*sin(s);h*s];

R = rotation_around_r([1 0 0], -pi/2);
fr1 = [0;0;r] + R*fr;
fr2 = [0;0;r] + [r*cos(s); h*s; r*sin(s)];

x = linspace(0, 1, 100);
fr1 = subs(fr1, {s, r, h}, {4*pi*t, 0.4, 0.3});
fr1 = subs(fr1, {t}, {x});
fr2 = subs(fr2, {s, r, h}, {4*pi*t, 0.4, 0.3});
fr2 = subs(fr2, {t}, {x});
view(3)
hold on
% plot3(fr1(1, :), fr1(2, :), fr1(3, :))
plot3(fr2(1, :), fr2(2, :), fr2(3, :))
hold off
grid on



