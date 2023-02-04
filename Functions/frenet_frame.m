function [t_s, n_s, b_s, k_s, tau_s] = frenet_frame(p_s, s)
    % frenet_frame - Compute the Frenet reference frame
    %
    % [t_s, n_s, b_s, k_s, tau_s] = frenet_frame(p_s, s)
    %
    % input:
    %   p_s - path in function of s
    %   s - the symbol of the function
    %
    % output:
    %   t_s - tangential component
    %   n_s - normal (or centripet) component in the osculating plane (take
    %         3 points and build a plane. After that converge the boundary
    %         point to the middle point and we have a osculating plane on
    %         that point)
    %   b_s - binormal component (right-hand rule between t and n)
    %   k_s - curvature, which is 1/radius of curvature
    %   tau_s - torsion

    p_s_p = simplify(diff(p_s, s));
    p_s_s = simplify(diff(p_s_p, s));
    p_s_t = simplify(diff(p_s_s, s));

    t_s = simplify(p_s_p/norm(p_s_p));
    t_s_p = simplify(diff(t_s, s));
    
    n_s = simplify(t_s_p/norm(t_s_p));

    b_s = simplify(cross(t_s, n_s));

%     n_s = simplify(cross(p_s_p, cross(p_s_s, p_s_p))/(norm(p_s_p)*norm(cross(p_s_s, p_s_p))));
%     b_s = simplify(cross(p_s_p, p_s_s)/norm(cross(p_s_p, p_s_s)));
    
    k_s = simplify(norm(cross(p_s_p, p_s_s))/(norm(p_s_p)^3));
    tau_s = simplify((p_s_p'*(cross(p_s_s, p_s_t)))/(norm(cross(p_s_p, p_s_p))^2));
end

