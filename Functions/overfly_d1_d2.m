function [d1, d2] = overfly_d1_d2(d_T, v1, v2)
    % overfly_d1_d2 - compute d1 and d2 in the transition overfly
    %
    % [d1, d2] = overfly_d1_d2(d_T, v1, v2)
    %
    % input:
    %   d_T - transition time
    %   v1 - initial velocity
    %   v2 - final velocity
    %
    % output:
    %   d1 - distance from the initial transition to the middle point
    %   d2 - distance from the middle point to the final transition
    
    d1 = (v1*d_T)/2;
    d2 = (v2*d_T)/2;
end

