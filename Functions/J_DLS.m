function [J_dls, q_dot] = J_DLS(J, lambda, m, v)
    % J_DLS - Compute the damped least squares method
    %
    % [J_dls, q_dot] = J_DLS(J, lambda, m, v)
    %
    % input:
    %   J - jacobian matrix
    %   lambda - damping factor
    %   m - number of coordinates of the end effector (position + orientation)
    %   v - vector of end effector velocities
    %
    % output:
    %   J_dls - Damped Least Squares Jacobian
    %   q_dot - joint velocities
    J_dls = J'*inv((lambda*eye(m) + J*J'));
    q_dot = J_dls*v;
end

