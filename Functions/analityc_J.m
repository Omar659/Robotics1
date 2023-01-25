function r_dot = analityc_J(fr, q, q_dot)
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

    J_r = jacobian(fr, q);
    r_dot = J_r * q_dot;
end

