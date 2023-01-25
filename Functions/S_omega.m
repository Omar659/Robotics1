function S = S_omega(omega)
    % S_omega - It is a skew simmetric matrix of the angular velocities of
    % the reference frame
    %
    % S = S_omega(omega)
    %
    % input:
    %   omega - A vector 3x1 with the three angular velocity
    %
    % output:
    %   S - Skew simmetric matrix 3x3 of the angular velocities
    S = [0 -omega(3) omega(2);
         omega(3) 0 -omega(1);
         -omega(2) omega(1) 0];
end

