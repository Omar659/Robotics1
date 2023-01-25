function R_dot = compute_R_dot(S, R)
    % compute_R_dot - Compute the derivative with respect the time of the
    % rotation matrix R:
    % R_dot = S(omega)*R
    %
    % R_dot = compute_R_dot(S, R)
    %
    % input:
    %   S - Skew simmetric matrix 3x3 of the angular velocities
    %   R - A 3x3 rotation matrix
    %
    % output:
    %   R_dot - Derivative with respect the time of the rotation matrix R
    R_dot = S*R;
end

