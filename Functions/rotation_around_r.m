function Rtr = rotation_around_r(r_m, theta)
    syms t rx ry rz
    Sr = [0 -rz ry;
          rz 0 -rx;
          -ry rx 0];
    r = [rx ry rz]';
    Rtr = mtimes(r, r') + (eye(3) - mtimes(r, r')) * cos(t) + Sr * sin(t);
    Rtr = subs(Rtr, {rx, ry, rz, t}, {r_m(1), r_m(2), r_m(3), theta});
end