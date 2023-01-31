function robot_animation(q_history, joint_positions, r_d, q, joint_types, speed)
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

    traj = [];
    for j = 1:size(q_history, 2)
        clf
        view(3);
        joint_pos = joint_positions;
        for i = 1:length(q)
            joint_pos = simplify(subs(joint_pos, {q(i)}, {q_history(i, j)}));
        end
        hold on
        traj = [traj joint_pos(:, end)];
        line(traj(1,:), traj(2,:), traj(3,:), LineWidth=4, Color=[1,0,0])
        line(joint_pos(1,:), joint_pos(2,:), joint_pos(3,:), LineWidth=4, Color=[0,0,1])
        scatter3(r_d(1), r_d(2), r_d(3), "black", "filled", "o")
        xlim([-1 1])
        ylim([-1 1])
        zlim([0 1])
        grid on
        pause(speed) %aspetta 0.1 secondi prima di aggiungere un nuovo punto
    end
end