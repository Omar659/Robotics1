clear all
clc

addpath("Functions\")
syms q1 q2 q3 px py pz real

d1 = 0.5;
fr = [q3 * cos(q2)* cos(q1);
      q3 * cos(q2)* sin(q1);
      d1 + q3 * sin(q2)];

q_0 = [0;0;1];
r_d = [1;1;1];
q = [q1; q2; q3];
q_3 = sqrt(px^2 + py^2 + (pz - d1)^2);
q_2_p = atan2((pz-d1)/q_3, sqrt(px^2 + py^2)/q_3);
q_2_s = atan2((pz-d1)/q_3, -sqrt(px^2 + py^2)/q_3);
q_1_p = atan2(py/cos(q_2_p), px/cos(q_2_p));
q_1_s = atan2(py/cos(q_2_s), px/cos(q_2_s));
q_des = simplify([q_1_p q_1_s;
                  q_2_p q_2_s;
                  q_3 q_3]);
q_des = subs(q_des, {px, py, pz}, {1, 1, 1});
disp(vpa(q_des, 5))

newton_method(fr, q_0, r_d, q, 15, 5, 6, q_des, 4)
t = 0;
dt = 0.1;
x = [1 2 3 4 5];
y = [2 4 6 8 10];
z = [3 6 9 12 15];
scatter3(x,y,z) % con scatter3
plot3(x,y,z) % con plot3
grid on % attiviamo la griglia
xlabel('X') % etichetta per l'asse x
ylabel('Y') % etichetta per l'asse y
zlabel('Z') % etichetta per l'asse z
% while(1)
%    x = cos(t);
%    y = sin(t);
%    z = t;
%    plot3(x,y,z,'o')
%    axis([-1 1 -1 1 0 10]) % impostiamo l'area di visualizzazione del grafico
%    grid on % attiviamo la griglia
%    drawnow %aggiorna il grafico in modo dinamico
%    t = t + dt; %aggiorniamo il parametro
%    pause(0.1) %aspetta 0.1 secondi prima di aggiungere un nuovo punto
% end