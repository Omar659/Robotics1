function r = compute_r(R,t)
    if t == 0
        r = 'no solution';
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
    else
        RR = R - R';
        r = [RR(3,2) RR(1,3) RR(2,1)]'/2*sin(t);
    end
end