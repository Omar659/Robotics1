function r = compute_r(R,t)
    % compute_r - Compute the unitary vector R around the matrix R is
    % rotated
    %
    % sintax: r = compute_r(R,t)
    %
    % input:
    %   R - A 3x3 rotation matrix
    %   t - The rotation angle
    %
    % output:
    %   check - Return the unitary vector r of the rotation

    % Singularity
    if t == 0
        r = 'no solution';
    % Two solutions
    elseif t == pi || t == -pi
        r = [sqrt((R(1,1) + 1)/2) sqrt((R(2,2) + 1)/2) sqrt((R(3,3) + 1)/2)]';
        if R(1,2)/2 < 0
            r(1) = -r(1);
        end
        if R(1,3)/2 < 0
            r(2) = -r(2);
        end
        if R(2,3)/2 < 0
            r(3) = -r(3);
        end
        r = [r -r];
    % Standard solution
    else
        RR = R - R';
        r = (1/(2*sin(t)))*[RR(3,2); RR(1,3); RR(2,1)];
    end
end