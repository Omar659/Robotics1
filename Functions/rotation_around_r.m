function Rtr = rotation_around_r(r_m, theta)
    % rotation_around_r - Compute the rotation matrix around a fixed
    % unitary vector r
    %
    % Rtr = rotation_around_r(r_m, theta)
    %
    % input:
    %   r_m - A 3x1 vector along which we need to rotate
    %   theta - The angle of rotation
    %
    % output:
    %   Rtr - The 3x3 rotation matrix around r_m of angle theta

    syms t rx ry rz
    Sr = [0 -rz ry;
          rz 0 -rx;
          -ry rx 0];
    r = [rx ry rz]';
    Rtr = mtimes(r, r') + (eye(3) - mtimes(r, r')) * cos(t) + Sr * sin(t);
    Rtr = subs(Rtr, {rx, ry, rz, t}, {r_m(1), r_m(2), r_m(3), theta});
end