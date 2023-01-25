function j_A_i = dh_matrix(a_i, alpha_i, d_i, theta_i)
    % dh_matrix - Compute the Denavit-Hartenberg matrix, between the j-th
    % and i-th joints, where j = i-1. The common normal is directed in the
    % direction of joint_i x joint_i-1 in the case of incident axis
    %
    % j_A_i = dh_matrix(a_j, alpha_j, d_i, theta_i)
    %
    % input:
    %   a_i - distance from z_i-1 to z_i measured along x_i axis.
    %   alpha_i - angle from z_i-1 to z_i measured about x_i axis.
    %   d_i - distance from x_i-1 to x_i measured along z_i-1 axis.
    %   theta_i - angle from x_i-1 to x_i measured along z_i-1 axis.
    %
    % output:
    %   j_A_i - The 4x4 Denavit-Hartenberg matrix

    j_A_ip = homogeneous_T(rotation_around_r([0 0 1], theta_i), [0;0;d_i]);
    ip_A_i = homogeneous_T(rotation_around_r([1 0 0], alpha_i), [a_i;0;0]);
    j_A_i = j_A_ip * ip_A_i;
end

