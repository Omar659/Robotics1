function [J_pinv, q_dot, v_feasible] = pinv_J(J, v)
    % pinv_J - Compute the pseudoinverse method
    %
    % [J_pinv, q_dot, v_feasible] = pinv_J(J, v)
    %
    % input:
    %   J - jacobian matrix
    %   v - vector of end effector velocities
    %
    % output:
    %   J_pinv - Pseusoinverse Jacobian
    %   q_dot - joint velocities
    %   v_feasible - vector of the feasibile end effector velocities
    
    J_pinv = pinv(J);
    q_dot = J_pinv*v;
    v_feasible = J*q_dot;
end

