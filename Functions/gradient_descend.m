function [q_history, norm_error_history] = gradient_descend(fr, q_0, r_d, q, alpha, k, eps, eps_q, q_des, fig_speed)
    % gradient_descend - Compute the Gradient Descent method to reach a
    % specific desired configuration
    %
    % [q_history, norm_error_history] = gradient_descend(fr, q_0, r_d, q, alpha, k, eps, eps_q, q_des, fig_speed)
    %
    % input:
    %   fr - Cartesian position of the EE in symbols
    %   q_0 - Initial configuration
    %   r_d - Desired cartesian position of the EE
    %   q - Vector of symbols of the configuration
    %   alpha - Step size
    %   k - Max iteration
    %   eps - Used as stopping criteria. If the cartesian error is lower
    %   than 10^-eps the algorithm stop with a success
    %   eps_q - Used as stopping criteria. If the cartesian error is lower
    %   than 10^-eps the algorithm stop with a success
    %   q_des - desired configuration
    %   fig_speed - amimation speed, 0 no animation, <0 no graph plot
    %
    % output:
    %   q_history - All configuration during the algorithm
    %   norm_error_history - All error norm during the algorithm

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

    if fig_speed>=0
        subplot(2,4,1);
        plot_errs_joints(norm_error_history, fig_speed, strcat('Gradient method with constant step=', num2str(alpha)), 'norm of Cartesian position error [m]')
    
        subplot(2,4,2);
        plot_errs_joints(error_history(1,:), fig_speed, strcat('Gradient method with constant step=', num2str(alpha)), 'ex [m]')
    
        subplot(2,4,3);
        plot_errs_joints(error_history(2,:), fig_speed, strcat('Gradient method with constant step=', num2str(alpha)), 'ey [m]')
    
        subplot(2,4,4);
        plot_errs_joints(error_history(3,:), fig_speed, strcat('Gradient method with constant step=', num2str(alpha)), 'ez [m]')
    
        subplot(2,4,5);
        plot_errs_joints(q_history(1,:), fig_speed, strcat('Gradient method with constant step=', num2str(alpha)), 'q1 [rad]', q_des, 1)
    
        subplot(2,4,6);
        plot_errs_joints(q_history(2,:), fig_speed, strcat('Gradient method with constant step=', num2str(alpha)), 'q2 [rad]', q_des, 2)
    
        subplot(2,4,7);
        plot_errs_joints(q_history(3,:), fig_speed, strcat('Gradient method with constant step=', num2str(alpha)), 'q3 [rad]', q_des, 3)
    end
end