addpath("Functions/") % Linux
% addpath("Functions\") % Windows

%%
clear all
close all
clc

DH_table = [0.5,  0,     0,   pi/2;
            0,  pi/2,    0.5,     0;
            0.5,  0,       0,  pi/4];
joint_types = ["r" "r" "r" "ee"];

A_01 = dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4));
A_12 = dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4));
A_23 = dh_matrix(DH_table(3,1), DH_table(3,2), DH_table(3,3), DH_table(3,4));
A_02 = A_01*A_12;
A_03 = A_01*A_12*A_23;
O_A_i = [eye(4) A_01 A_02 A_03];
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

syms q1 q2 q3 L M N real
DH_table = [L,    0,     0, q1;
            0, pi/2,     M, q2;
            N,    0,     0, q3];

A_01 = dh_matrix(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4))
A_12 = dh_matrix(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4))
A_23 = dh_matrix(DH_table(3,1), DH_table(3,2), DH_table(3,3), DH_table(3,4))

O_p_O3 = simplify(A_01*(A_12*(A_23*[0;0;0;1])))

%%

clear all
close all
clc

syms q1 q2 q3 L M N real

f_r = [L*cos(q1) + N*cos(q1+q2)*cos(q3);
       L*sin(q1) + N*sin(q1+q2)*cos(q3);
       M + N*sin(q3)]

J = jacobian(f_r, [q1 q2 q3])

det_J = det(J)


J_s = simplify(subs(J, {q2}, {pi/2}))
rank_J_s = rank(J_s)

range_J_s = orth(J_s)

v = J_s*[-sin(q1); cos(q1); 0]

J_s_pinv = simplify(pinv(J_s))


range_J_s_pinv = orth(J_s_pinv)

qd = simplify(J_s_pinv*[-sin(q1); cos(q1); 0])


%%

clear all
close all
clc

syms q1 q2 q1d q2d q1dd q2dd real

f_r = [q2*cos(q1); q2*sin(q1)]
J = simplify(jacobian(f_r, [q1 q2]))
Jd = [-q2*cos(q1)*q1d, -sin(q1)*q1d;
      -q2*sin(q1)*q1d, cos(q1)*q1d]

pdd = simplify(J*[q1dd; q2dd] + Jd*[q1d; q2d])


A = [-q2*sin(q1), cos(q1); 
      q2*cos(q1), sin(q1)]
B = [q2*cos(q1)*q1d^2 + q2d*sin(q1)*q1d;
     q2*sin(q1)*q1d^2 - q2d*cos(q1)*q1d]

x = simplify((inv(A)*B))


%%

clear all
close all
clc


syms q1 q2 real
f_r = [cos(q1) + cos(q1 + q2);
       sin(q1) + sin(q1 + q2)]

J = simplify(jacobian(f_r, [q1 q2]))

det_J = simplify(det(J))

J_inv = simplify(inv(J))

syms k
vpa(subs(inv([-0.5 -0.9114; 0.5 -0.4114])*[0.5*k; 0.5], {k}, {4.2915}), 4)



%%
clear all
close all
clc

syms q1 q2 q1d q2d q1dd q2dd real

f_r = [q2*cos(q1); q2*sin(q1)]
J = simplify(jacobian(f_r, [q1 q2]))

J_T = J'

J_T_c = vpa(subs(J_T, {q1, q2}, {pi/3, 1.5}), 4)




