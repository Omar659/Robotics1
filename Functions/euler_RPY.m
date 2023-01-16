function R = euler_RPY(ax1, ax2, ax3, ang1, ang2, ang3, moving)
    % euler_RPY - Compute the rotation matrix with 3 moving axis or fixed
    % axis (e.g. Roll-Pitch-Yaw)
    %
    % R = euler_RPY(ax1, ax2, ax3, ang1, ang2, ang3, moving)
    %
    % input:
    %   ax1 - First axis of the rotation
    %   ax2 - Second axis of the rotation
    %   ax3 - Third axis of the rotation
    %   ang1 - First angle of the rotation
    %   ang2 - Second angle of the rotation
    %   ang3 - Third angle of the rotation
    %   moving - boolean, if true I consider the axis as moving axis (so
    %   the order of multiplication is in order 1->2->3), if false I
    %   consider the axis as fixed (so the order of multiplication is
    %   inverted 3->2->1)
    %
    % output:
    %   R - The rotation matrix
    
    R1 = rotation_around_r(ax1, ang1);
    R2 = rotation_around_r(ax2, ang2);
    R3 = rotation_around_r(ax3, ang3);
    if moving
        R = R1 * R2 * R3;
    else
        R = R3 * R2 * R1;
    end
end