function T_RPY = derivative_RPY(syms)
    % derivative_RPY - Compute the detivative with respect the time of the
    % rotation matrix Roll-Pitch-Yaw
    %
    % T_RPY = derivative_RPY(syms)
    %
    % input:
    %   syms - Name of the angles
    %
    % output:
    %   T_RPY - Derivative with respect the time of the RPY matrix
    
    R_1 = rotation_around_r([0 0 1], syms(3));
    R_2 = rotation_around_r([0 1 0], syms(2));
    col_3_m = eye(3);
    col_2_m = R_1;
    col_1_m = R_1*R_2;
    T_RPY = [col_1_m(:, 1) col_2_m(:, 2) col_3_m(:,3)];
end