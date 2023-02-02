function [T_s, T] = bang_cost_bang(L, v_max, a_max)
    % bang_cost_bang - find T_s and T
    %
    % [T_s, T] = bang_cost_bang(L, v_max, a_max)
    %
    % input:
    %   L - norm of pf - pi, path length ||pf - pi||
    %   v_max - scalar representing the max velocity
    %   a_max - scalar representing the max accelleration
    %
    % output:
    %   T_s - time necessary to perform bang 
    %   T - total time
    
    % v_max*(T - T_s) = L
    T_s = v_max/a_max;
    T = (L*a_max + v_max^2)/(a_max * v_max);
end

