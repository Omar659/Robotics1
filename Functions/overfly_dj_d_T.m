function [d_j, d_T] = overfly_dj_d_T(d_i, v_i, v_j)
    % overfly_dj_d_T - compute d_j and d_T from d_i, v_i, v_j
    %
    % [d_j, d_T] = overfly_dj_d_T(d_i, v_i, v_j)
    %
    % input:
    %   d_i - distance from the middle point, A' if i = 1, C' otherwise
    %   v_i - initial velocity if i = 1, final velocity otherwise
    %   v_j - final velocity if i = 1, intial velocity otherwise
    %
    % output:
    %   d_j - distance from the middle point, C' if i = 1, A' otherwise
    %   d_T - transition time
    
    d_T = (2*d_i)/v_i;
    d_j = (d_i*v_j)/v_i;
end

