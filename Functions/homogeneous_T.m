function a_T_b = homogeneous_T(a_R_b, a_p_ab)
    a_T_b = [a_R_b(1,:) a_p_ab(1);
             a_R_b(2,:) a_p_ab(2);
             a_R_b(3,:) a_p_ab(3);
             0 0 0 1];
end

