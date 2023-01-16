function b_T_a = inverse_T(a_T_b)
    % inverse_T - Compute the inverse of the homogeneous transformation matrix
    %
    % b_T_a = inverse_T(a_T_b)
    %
    % input:
    %   a_T_b - A 4x4 homogeneous transformation matrix
    %
    % output:
    %   b_T_a - The 4x4 inverse matrix of a_T_b

    a_R_b = a_T_b(1:3,1:3);
    b_R_a = a_R_b';
    a_p_ab = a_T_b(1:3, 4);
    b_P_ba = -b_R_a * a_p_ab;
    b_T_a = [b_R_a(1,:) b_P_ba(1);
             b_R_a(2,:) b_P_ba(2);
             b_R_a(3,:) b_P_ba(3);
             0 0 0 1];
end

