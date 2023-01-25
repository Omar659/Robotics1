function S = compute_S(R_dot, R)
    % compute_S - It is a skew simmetric matrix of the angular velocities of
    % the reference frame using the R and R_dot matrices: 
    % S(omega) = R_dot*R'
    %
    % S = compute_S(R_dot, R)
    %
    % input:
    %   R_dot - Derivative with respect the time of the rotation matrix R
    %   R - A 3x3 rotation matrix
    %
    % output:
    %   S - Skew simmetric matrix 3x3 of the angular velocities
    S = R_dot*R';
end

