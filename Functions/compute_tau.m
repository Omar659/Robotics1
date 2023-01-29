function tau = compute_tau(J,F)
    % compute_tau - Compute tau (torques and forces) that balance F_e
    %
    % sintax: tau = compute_tau(J,F)
    %
    % input:
    %   J - jacobian matrix
    %   F - forces vector
    %
    % output:
    %   tau - Return tau

    tau = J'*F;
end