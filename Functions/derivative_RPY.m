function T_RPY = derivative_RPY(ax1, ax2, ax3, syms)
    % derivative_RPY - Compute one of the 12 combinations of the derivative 
    % with respect the time of the rotation matrix Roll-Pitch-Yaw
    %
    % T_RPY = derivative_RPY(syms)
    %
    % input:
    %   ax1 - Index of first rotation, fixed axis
    %   ax2 - Index of second rotation, fixed axis
    %   ax3 - Index of third rotation, fixed axis
    %   syms - Name of the angles
    %
    % output:
    %   T_RPY - Derivative with respect the time of the RPY matrix

    % rotation axis creation
    % e.g. X Y Z
    r_2 = zeros(3,1);  
    r_2(ax2) = 1; % e.g. 0 1 0 
    r_3 = zeros(3,1);
    r_3(ax3) = 1; % e.g. 0 0 1
    % Rotation around the last two axis (moving axis are read backwards)
    % e.g. Z Y' X'' 
    R_1 = rotation_around_r(r_3, syms(3));
    R_2 = rotation_around_r(r_2, syms(2));
    % ax3 column of the identity matrix
    % e.g. Z -> Z = third column
    col_3_m = eye(3);
    col_3_m = col_3_m(:, ax3);
    % ax2 column of the first rotation matrix R_1 (moving axis)
    % e.g. Y' -> Y = second column
    col_2_m = R_1(:, ax2);
    % ax1 column of the (R_1 * R_2) rotation matrix (moving axis)
    % e.g. X'' -> X = first column
    col_1_m = R_1 * R_2;
    col_1_m = col_1_m(:, ax1);
    % the three columns together  
    T_RPY = [col_1_m col_2_m col_3_m];
end