function gradient_descend(fr, q_0, r_d, q, alpha, k, eps, eps_q, q_des, fig_speed)
    % angles_from_RPY - return the angle theta (pitch), phi (roll) and 
    % psi (yaw) from a Roll-Pitch-Yaw rotation matrix R
    %
    % sintax: [sol1, sol2] = angles_from_RPY(R)
    %
    % input:
    %   R - Roll-Pitch-Yaw rotation matrix R
    %
    % output:
    %   [sol1, sol2] - The two solutione since the atan2 function of theta
    %   take as cos a plus/manus value, resulting in two output
    disp('GRADIENT DESCEND')
    q_k = q_0;
    q_history = [];
    error_history = [];
    norm_error_history = [];
    J = jacobian(fr, q);
    J_T = J';
    disp('STEP0')
    disp(q_k)
    for ep = 1:k
        q_history(:, end + 1) = q_k;
        
        fr_q_k = fr;
        J_T_k = J_T;
        for i = 1:length(q_k)
            fr_q_k = subs(fr_q_k, {q(i)}, {q_k(i)});
            J_T_k = subs(J_T_k, {q(i)}, {q_k(i)});            
        end

        cartesian_error_k = r_d - fr_q_k;
        error_history(:, end + 1) = cartesian_error_k;
        norm_error_history(:, end + 1) = norm(cartesian_error_k);
        if norm(cartesian_error_k) <= 10^-eps
            disp('SOLUTION FOUND, CARTESIAN ERROR LESS THAN eps')
            break
        end
        q_k1 = vpa(simplify(q_k + alpha*J_T_k * cartesian_error_k),eps_q);
        algorithm_increment_k = norm(q_k1 - q_k);
        if algorithm_increment_k <= 10^-eps_q
            disp('SOLUTION FOUND, ALGORITHM INCREMENT LESS THAN eps_q')
            break
        end
        
        q_k = q_k1;
        disp(strcat('STEP', num2str(ep)))
        disp(q_k)
    end
    subplot(2,4,1);
    plot_errs_joints(norm_error_history, fig_speed, strcat('Gradient method with constant step=', num2str(alpha)), 'norm of Cartesian position error [m]')

    subplot(2,4,2);
    plot_errs_joints(error_history(1,:), fig_speed, strcat('Gradient method with constant step=', num2str(alpha)), 'ex [m]')

    subplot(2,4,3);
    plot_errs_joints(error_history(2,:), fig_speed, strcat('Gradient method with constant step=', num2str(alpha)), 'ey [m]')

    subplot(2,4,4);
    plot_errs_joints(error_history(3,:), fig_speed, strcat('Gradient method with constant step=', num2str(alpha)), 'ez [m]')

    subplot(2,4,5);
    plot_errs_joints(q_history(1,:), fig_speed, strcat('Gradient method with constant step=', num2str(alpha)), 'q1 [rad]', q_des)

    subplot(2,4,6);
    plot_errs_joints(q_history(2,:), fig_speed, strcat('Gradient method with constant step=', num2str(alpha)), 'q2 [rad]', q_des)

    subplot(2,4,7);
    plot_errs_joints(q_history(3,:), fig_speed, strcat('Gradient method with constant step=', num2str(alpha)), 'q3 [rad]', q_des)
end