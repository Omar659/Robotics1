function j_A_i = dh_matrix(alpha, a, d, theta)
    % dh_matrix - Compute the Denavit-Hartenberg matrix, between the j-th
    % and i-th joints, where j = i-1.
    %
    % j_A_i = dh_matrix(alpha, a, d, theta)
    %
    % input:
    %   alpha - angle between z_j and z_i, to calculate look the x_i axis
    %   a - common normal length between j-th and i-th joints
    %   d - distance between orgins of z_j and z_i
    %   theta - is the angle between x_j and x_i, to calculate look at z_j
    %   axis
    %
    % output:
    %   j_A_i - The 4x4 Denavit-Hartenberg matrix

    j_A_ip = homogeneous_T(rotation_around_r([0 0 1], theta), [0;0;d]);
    ip_A_i = homogeneous_T(rotation_around_r([1 0 0], alpha), [a;0;0]);
    j_A_i = j_A_ip * ip_A_i;
end

