function [a_v_a, a_w] = transformations_among_v(a_R_b, b_r_ba, b_v_b, b_w)
    % transformations_among_v - Compute transformation among generalized velocities
    %
    % sintax: [a_v_a, a_w] = transformations_among_v(a_R_b, b_r_ba, b_v_b, b_w)
    %
    % input:
    %   a_R_b - rotation matrix of frame B wrt frame A
    %   b_r_ba - vector from origin of frame B to origin of frame A wrt
    %   frame B
    %   b_v_b - linear velocity of frame B wrt frame B 
    %   b_w - angular velocity of frame B wrt frame B
    % output:
    %   a_v_a - linear velocity of frame A wrt frame A
    %   a_w - angular velocity of frame A wrt frame A
    
    J_ba = [a_R_b -a_R_b*S_omega(b_r_ba);
            zeros(3) a_R_b];
    r = J_ba * [b_v_b;
                b_w];
    a_v_a = r(1:3, :);
    a_w = r(4:6, :);
end