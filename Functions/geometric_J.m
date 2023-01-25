function [v_E, w_E, J] = geometric_J(DH_table, joint_types, q_dot)
    % analityc_J - Compute the analytic Jacobian obtained by time  
    % differentiation
    %
    % input:
    %   fr - Is the matrix [p; phi] = r
    %   q - Vector of symbols of the configuration
    %   q_dot - Vector of symbols of the joints velocities (derivative of
    %   q)
    %
    % output:
    %   r_dot - Is the matrix [p_dot; phi_dot]
    
    % j = i-1
    O_T_E = eye(4);
    for i=size(DH_table, 1):-1:1
        joint_i = DH_table(i, :);
        j_T_i = dh_matrix(joint_i(1), joint_i(2), joint_i(3), joint_i(4));
        O_T_E = j_T_i * O_T_E;
    end

    % j = i-1
    J = zeros(6,1);
    for i=1:size(DH_table, 1)
        z_j = [0;0;1];
        % u = k-1
        O_T_j = eye(4);
        for k=i-1:-1:1
            joint_k = DH_table(k, :);
            u_T_k = dh_matrix(joint_k(1), joint_k(2), joint_k(3), joint_k(4));
            u_R_k = u_T_k(1:3, 1:3);
            z_j = u_R_k * z_j;
            O_T_j = u_T_k * O_T_j;
        end
        if joint_types(i) == 'r'
            p_jE = O_T_E(1:3, 4) - O_T_j(1:3, 4);
            J_Li = cross(z_j, p_jE);
            J_Ai = z_j;
        else
            J_Li = z_j;
            J_Ai = zeros(3,1);
        end
        J_L_A_i = [J_Li;J_Ai];
        J = [J J_L_A_i];
    end
    J = J(:, 2:end);
    result = J * q_dot;
    v_E = result(1:3);
    w_E = result(4:6);
end

