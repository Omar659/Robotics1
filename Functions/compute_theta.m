function [t1, t2, t3] = compute_theta(R)
    % compute_theta - Compute the theta angle of the rotation matrix R
    %
    % [t1, t2, t3] = compute_theta(R)
    %
    % input:
    %   R - A 3x3 rotation matrix
    %
    % output:
    %   t1 - Return the theta angle calculated with the acos function (not
    %   optimal)
    %   t2 - Return the theta angle calculated with atan2 function with a
    %   positive square root as sine
    %   t2 - Return the theta angle calculated with atan2 function with a
    %   negative square root as sine

    % With acos function
    t1 = acos((trace(R)-1)/2);
    c = (trace(R)-1)/2;
    RR = R - R';
    rx = RR(2, 3);
    ry = RR(1, 3);
    rz = RR(1, 2);
    s = (1/2)*sqrt(rx^2 + ry^2 + rz^2);
    % With atan2 function
    t2 = atan2(s, c);
    t3 = atan2(-s, c);
end