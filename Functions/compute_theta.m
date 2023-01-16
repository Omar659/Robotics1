function [t1, t2, t3] = compute_theta(R)
    t1 = acos((trace(R)-1)/2);
    c = (trace(R)-1)/2;
    RR = R - R';
    rx = RR(2, 3);
    ry = RR(1, 3);
    rz = RR(1, 2);
    s = (1/2)*sqrt(rx^2 + ry^2 + rz^2);
    t2 = atan2(s, c);
    t3 = atan2(-s, c);
end