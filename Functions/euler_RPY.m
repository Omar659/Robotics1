function R = euler_RPY(ax1, ax2, ax3, ang1, ang2, ang3, moving)
    R1 = rotation_around_r(ax1, ang1);
    R2 = rotation_around_r(ax2, ang2);
    R3 = rotation_around_r(ax3, ang3);
    if moving
        R = R1 * R2 * R3;
    else
        R = R3 * R2 * R1;
    end
end