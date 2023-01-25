function mobility_analysis(J, r, m, n , q, q_d, q_v)
    % mobility_analysis - Compute the mobility analysis. 
    %
    % mobility_analysis(J, r, m, n , q, q_d, q_v)
    %
    % input:
    %   J - Jacobian of fr(q)
    %   r - end effector position + orientation
    %   m - number of elements of r
    %   n - number of joints
    %   q - symbols of the configuration vector
    %   q_d - symbols of the velocities vector
    %   q_v - values for configuration q (case study)
    
    J_r = subs(J, {q(1), q(2), q(3)}, {q_v(1), q_v(2), q_v(3)});
    display(J_r)
    J_r_T = J_r';
    
    ro_J = rank(J_r);
    ro_J_T = rank(J_r_T);
    disp(strcat(strcat('rank of J_r should be:', num2str(m)), ', otherwise loss of mobility'))
    disp(ro_J);
    disp('rank of J_r transposed')
    disp(ro_J_T);
    
    range_J = vpa(orth(J_r), 4);
    range_J_T = vpa(orth(J_r_T), 4);
    disp('range of J_r')
    disp(range_J);
    disp('range of J_r transposed')
    disp(range_J_T);
    
    N_J = null(J_r);
    N_J_T = null(J_r_T);
    disp('nullspace of J_r')
    display(N_J);
    disp('nullspace of J_r transposed')
    display(N_J_T);
    
    disp(strcat('check dimensionality range J + Nullspace J_T should be:',num2str(m)))
    disp(size(range_J, 2) + size(N_J_T, 2))
    
    disp(strcat('check dimensionality range J_T + Nullspace J should be:',num2str(n)))
    disp(size(range_J_T, 2) + size(N_J, 2))
    
    if size(N_J,2) > 0
        r_d = subs(r, {q(1), q(2), q(3), q_d(1), q_d(2), q_d(3)}, {q_v(1), q_v(2), q_v(3), N_J(1,1), N_J(2,1), N_J(3,1)});
        display(r_d)
    else
        disp('no nullspace')
    end
end

