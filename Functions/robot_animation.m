function robot_animation(q_history, joint_positions, r_d, q, speed)
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
    
    [X,Y,Z] = sphere;
    r = 0.025;
    X2 = X * r;
    Y2 = Y * r;
    Z2 = Z * r;
    traj_x = [];
    traj_y = [];
    traj_z = [];
    for j = 1:size(q_history, 2)
        clf
        view(3);
        joint_pos = joint_positions;
        for i = 1:length(q)
            joint_pos = simplify(subs(joint_pos, {q(i)}, {q_history(i, j)}));
        end
        hold on
        plot3(joint_pos(1,:), joint_pos(2,:), joint_pos(3,:), LineWidth=3, Color=[0,0,1])
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

        for p = 1:size(joint_positions, 2)
            surf(X2 + double(joint_pos(1,p)), Y2 + double(joint_pos(2,p)), Z2 + double(joint_pos(3,p)), FaceColor=[0.5 0.5 0.5], EdgeColor='none')
        end
        surf(X2 + double(r_d(1)), Y2 + double(r_d(2)), Z2 + double(r_d(3)), EdgeLighting="gouraud", EdgeColor="none")
        xlim([-1-r 1+r])
        ylim([-1-r 1+r])
        zlim([0-r 1+r])
        grid on
        hold off
        pause(speed) %aspetta 0.1 secondi prima di aggiungere un nuovo punto
    end
end