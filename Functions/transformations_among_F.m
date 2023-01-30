function [b_f_b, b_m] = transformations_among_F(b_R_a, b_r_ba, a_f_a, a_m)
    % transformations_among_v - Compute transformation among generalized velocities
    %
    % sintax: [a_v_a, a_w] = transformations_among_v(a_R_b, b_r_ba, b_v_b, b_w)
    %
    % input:
    %   b_R_a - rotation matrix of frame A wrt frame B
    %   b_r_ba - vector from origin of frame B to origin of frame A wrt
    %   frame B
    %   a_f_a - force of frame A wrt frame A 
    %   a_m - momentum of frame A wrt frame A
    % output:
    %   b_f_b - force of frame B wrt frame B 
    %   b_m - momentum of frame B wrt frame B
    
    J_ba_T = [b_R_a zeros(3) 
              S_omega(b_r_ba)*b_R_a b_R_a];
    r = J_ba_T * [a_f_a;
                  a_m];
    b_f_b = r(1:3, :);
    b_m = r(4:6, :);
end