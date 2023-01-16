function a_T_b = homogeneous_T(a_R_b, a_p_ab)
    % homogeneous_T - Compute the homogeneus transformation matrix
    %
    % a_T_b = homogeneous_T(a_R_b, a_p_ab)
    %
    % input:
    %   a_R_b - A 3x3 rotation matrix of the referance frame B wrt the
    %   reference frame A
    %   a_p_ab - The 1x3 vector from the origin of the reference frame A to
    %   the origin of the reference frame B wrt the reference frame A
    %
    % output:
    %   a_t_b - The homogeneus 4x4 transformation matrix of B wrt A

    a_T_b = [a_R_b(1,:) a_p_ab(1);
             a_R_b(2,:) a_p_ab(2);
             a_R_b(3,:) a_p_ab(3);
             0 0 0 1];
end