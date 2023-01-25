function [v_E, w_E, J] = geometric_J(DH_table, joint_types, q_dot)
    % geometric_J - Compute the geometric Jacobian obtained by the Denavit
    % Hartenberg table and joint types. [v_E; w_E] = [J_L; J_A]*q_dot
    %
    % [v_E, w_E, J] = geometric_J(DH_table, joint_types, q_dot)
    %
    % input:
    %   DH_table - The Denavit Hartenberg table
    %   joint_types - Ordered array of the joint types: "r" = revolute; 
    %   "p" = prismatic
    %   q_dot - Vector of symbols of the joints velocities (derivative of
    %   q)
    %
    % output:
    %   v_E - (Cartesian) Linear velocities vector of the end effector
    %   (first 3 rows of the result J*q_dot)
    %   w_E - Angular velocities vector of the end effector (second 3 rows 
    %   of the result J*q_dot)
    %   J - Geometric Jacobian
    
    % Homogeneous transformation from the origin to the end effector.
    % Starting from the Identity I computed this product:
    % O_T_E = O_T_1 * (1_T_2 * (... * (i-2_T_i-1 * (i-1_T_E * I) ) ) )
    O_T_E = eye(4);
    % j = i-1
    for i=size(DH_table, 1):-1:1
        joint_i = DH_table(i, :);
        j_T_i = dh_matrix(joint_i(1), joint_i(2), joint_i(3), joint_i(4));
        O_T_E = j_T_i * O_T_E;
    end

    % In this loop I compute J as [J_Li; J_Ai] where J_Li is the geometric
    % linear jacobian and J_Ai is the geometri angular jacobian
    J = zeros(6,1); % The first column will be removed
    % j = i-1
    for i=1:size(DH_table, 1)
        % The z-axis of the i-1 joint.
        z_j = [0;0;1];
        % Homogeneous transformation from the origin to the joint i-1
        O_T_j = eye(4);
        % u = k-1
        for k=i-1:-1:1
            joint_k = DH_table(k, :);
            u_T_k = dh_matrix(joint_k(1), joint_k(2), joint_k(3), joint_k(4));
            % Rotation matrix from the homogenous transformation
            u_R_k = u_T_k(1:3, 1:3);
            % Starting from the z-axis not rotated ([0; 0; 1]):
            % z_i-1 = O_R_1 * (1_R_2 * (... * (i-2_R_i-1 * I) ) )
            z_j = u_R_k * z_j;
            % Starting from the Identity I computed this product:
            % O_T_E = O_T_1 * (1_T_2 * (... * (i-2_T_i-1 * I) ) ) )
            O_T_j = u_T_k * O_T_j;
        end
        % If the joint is a revolute joint, J_Li = z_i-1 x p_jE 
        % and J_Ai = z_i-1
        if joint_types(i) == 'r'
            % p_i-1,E is the position point of the homogeneous transformation
            % from the origin to the end effector minus the homogeneous 
            % transformation from the origin to the i-1 joint
            p_jE = O_T_E(1:3, 4) - O_T_j(1:3, 4);
            J_Li = cross(z_j, p_jE);
            J_Ai = z_j;
        % Else the joint is a prismatic joint, J_Li = z_i-1 
        % and J_Ai = [0;0;0]
        else
            J_Li = z_j;
            J_Ai = zeros(3,1);
        end
        % Add the column of the geometric jacobians
        J_L_A_i = [J_Li;J_Ai];
        J = [J J_L_A_i];
    end
    J = J(:, 2:end);
    % Results
    result = J * q_dot;
    v_E = result(1:3);
    w_E = result(4:6);
end

