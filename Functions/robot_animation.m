function robot_animation(q_history, joint_positions, link_lenghts, r_d, q, joint_types, speed)
    % compute_theta - Compute the theta angle of the rotation matrix R
    %
    % [t1, t2, t3] = compute_theta(R)
    %
    % input:
    %   R - A 3x3 rotation matrix
    %
    % output:
    %   t1 - Return the theta angle calculated with the acos function (not
    %   optimal)
    %   t2 - Return the theta angle calculated with atan2 function with a
    %   positive square root as sine
    %   t2 - Return the theta angle calculated with atan2 function with a
    %   negative square root as sine
    
    import geom3d.*
    % Revolute joint
    [X,Y,Z] = sphere(10);
    r = 0.04;
    revolute_X = X * r;
    revolute_Y = Y * r;
    revolute_Z = Z * r;

    traj_x = [];
    traj_y = [];
    traj_z = [];
    
    % Crea le coordinate dei vertici del cubo
    syms L W H real
    cube_verteces = [-L/2 -W/2 -H/2; 
                      L/2 -W/2 -H/2; 
                      L/2  W/2 -H/2; 
                     -L/2  W/2 -H/2; 
                     -L/2 -W/2 H/2; 
                      L/2 -W/2 H/2; 
                      L/2  W/2  H/2; 
                     -L/2  W/2  H/2];
    % Crea le facce del cubo come una serie di indici dei vertici
    cube_faces = [1 2 3 4; 
                  2 6 7 3; 
                  4 3 7 8; 
                  1 5 8 4; 
                  1 2 6 5; 
                  5 6 7 8];
    prismatic_joint = cube_verteces;
    % Prismatic joint
    l = 3/2*r; % lunghezza
    w = 3/2*r; % larghezza
    h = 3/2*r; % altezza
    prismatic_joint = double(subs(prismatic_joint, {L, W, H}, {l, w, h}));

    % Cylinder radius
    r_start = r*2/3;
    r_finish = r_start*1/3;

    for j = 1:size(q_history, 2)
        clf
        view(3);
        joint_pos = joint_positions;
        for i = 1:length(q)
            joint_pos = simplify(subs(joint_pos, {q(i)}, {q_history(i, j)}));
        end
        hold on
        for i = 2:size(joint_pos, 2)
            d_i = joint_pos(:, i) - joint_pos(:, i-1);
            scale = link_lenghts(i-1);
            if joint_types(i) == "p"
                scale = abs(q_history(i, j));
            end
%             [x_c, y_c, z_c] = cylinder2([r_start/scale r_finish/scale], d_i, 10);
%             x_c = double(x_c*scale + joint_pos(1, i-1));
%             y_c = double(y_c*scale + joint_pos(2, i-1));
%             z_c = double(z_c*scale + joint_pos(3, i-1));
            [x_c, y_c, z_c] = cylinder2P([r_start r_finish],6,joint_pos(:, i-1)',joint_pos(:, i)');
%             [x_c, y_c, z_c] = cylinder2([r_start r_finish], d_i, 10, scale);
%             x_c = double(x_c + joint_pos(1, i-1));
%             y_c = double(y_c + joint_pos(2, i-1));
%             z_c = double(z_c + joint_pos(3, i-1));
            x_c = double(x_c);
            y_c = double(y_c);
            z_c = double(z_c);
            surf(x_c, y_c, z_c, FaceColor=[0 0.5 1], EdgeColor='none')
        end
%         plot3(joint_pos(1,:), joint_pos(2,:), joint_pos(3,:), LineWidth=4, Color=[0,0,1])
        xlim([-1-r 1+r])
        ylim([-1-r 1+r])
        zlim([0-r 1+r])
        grid on
            
        traj_x = [traj_x joint_pos(1,end)];
        traj_y = [traj_y joint_pos(2,end)];
        traj_z = [traj_z joint_pos(3,end)];
        plot3(traj_x, traj_y, traj_z, LineWidth=1, Color=[1,0,0])
        % devo mettere limite a minimo e massimo della storia per far
        % vederfe il plot
        xlim([-1-r 1+r])
        ylim([-1-r 1+r])
        zlim([0-r 1+r])
        grid on

%         for p = 1:(size(joint_positions, 2)-1)
% %             if joint_types(p) == "r"
%             d_p = joint_pos(:, p+1) - joint_pos(:, p);
%             [x_c, y_c, z_c] = cylinder2([1 1], d_p, 10);
%             x_c = double(x_c*r + joint_pos(1, p));
%             y_c = double(y_c*r + joint_pos(2, p));
%             z_c = double(z_c*r + joint_pos(3, p));
%             surf(x_c, y_c, z_c, FaceColor=[.9 .9 .9], EdgeColor='none')
%             surf(revolute_X + double(joint_pos(1,p)), revolute_Y + double(joint_pos(2,p)), revolute_Z + double(joint_pos(3,p)), ...
%                 FaceColor=[.9 .9 .9], EdgeColor='none')
%             else
%                 p_joint = prismatic_joint;
%                 p_joint(:,1) = p_joint(:,1) + double(joint_pos(1,p));
%                 p_joint(:,2) = p_joint(:,2) + double(joint_pos(2,p));
%                 p_joint(:,3) = p_joint(:,3) + double(joint_pos(3,p));
%                 patch(Faces = cube_faces, Vertices = p_joint, ...
%                     FaceColor=[.9 .9 .9], EdgeColor='none');
%             end
%         end
% 
%         if joint_types(end) == "r"
%             surf(revolute_X + double(r_d(1)), revolute_Y + double(r_d(2)), revolute_Z + double(r_d(3)), FaceAlpha=0)
%         else
%             p_joint = prismatic_joint;
%             p_joint(:,1) = p_joint(:,1) + double(r_d(1));
%             p_joint(:,2) = p_joint(:,2) + double(r_d(2));
%             p_joint(:,3) = p_joint(:,3) + double(r_d(3));
%             patch(Faces = cube_faces, Vertices = p_joint, FaceColor = [0.9 0.9 0.9], FaceAlpha=0);
%         end

        xlim([-1-r 1+r])
        ylim([-1-r 1+r])
        zlim([0-r 1+r])
        grid on
        % imposta l'angolo della luce
        lightangle(45, 30)        
        % imposta l'effetto di illuminazione
        lighting gouraud
        hold off
        pause(speed) %aspetta 0.1 secondi prima di aggiungere un nuovo punto
    end
end