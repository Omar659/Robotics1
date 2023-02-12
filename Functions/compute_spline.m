function [th, thd, thdd] = compute_spline(n_traj, hk, qk, vk)
    % compute_spline - Compute spline (up to acceleration, rest-to-rest)
    %
    % [th, thd, thdd] = compute_spline(n_traj, hk, qk, vk)
    %
    % input:
    %   n_traj - 
    %   hk - 
    %   qk - 
    %   vk - 
    %
    % output:
    %   th - 
    %   thd - 
    %   thdd - 
    
    syms t real
    
    th = [];
    thd = [];
    thdd = [];

    tk = 0;
    tau = [];
    for k = 1:n_traj
        tau = [tau; t-tk];
        tk = tk + hk(k);
    end
%     tau = [t, t-hk(1)];
    for k = 1:n_traj
        thk = sym(strcat("a", num2str(k), "0")) + ...
              sym(strcat("a", num2str(k), "1"))*tau(k) + ...
              sym(strcat("a", num2str(k), "2"))*tau(k)^2 + ...
              sym(strcat("a", num2str(k), "3"))*tau(k)^3;
        thk = subs(thk, {strcat("a", num2str(k), "0"), ...
                         strcat("a", num2str(k), "1")}, {qk(k), vk(k)});  
        th = [th; thk];
        thdk = sym(strcat("a", num2str(k), "1")) + ...
               2*sym(strcat("a", num2str(k), "2"))*tau(k) + ...
               3*sym(strcat("a", num2str(k), "3"))*tau(k)^2;
        thdk = subs(thdk, {strcat("a", num2str(k), "1")}, {vk(k)});   
        thd = [thd; thdk];
        thddk = 2*sym(strcat("a", num2str(k), "2")) + ...
                6*sym(strcat("a", num2str(k), "3"))*tau(k);
        thdd = [thdd; thddk];
    end

    a23 = [];
    for k = 1:n_traj
        A = [hk(k)^2 hk(k)^3;
             2*hk(k) 3*hk(k)^2];
        B = [qk(k+1) - qk(k) - vk(k)*hk(k);
             vk(k+1) - vk(k)];
        ak23 = inv(A)*B;
        a23 = [a23 ak23];
    end
    A_h = [];
    for k = 1:n_traj-1
        z_f = zeros(1, k-1);
        z_a = zeros(1, n_traj - k-1);
        A_h = [A_h; z_f hk(k+1) 2*(hk(k) + hk(k+1)) hk(k) z_a];
    end
    A_h = A_h(:, 2:end-1);

    B_hq1 = (3/(hk(1)*hk(2)))*((qk(3) - qk(2))*hk(1)^2 + (qk(2) - qk(1))*hk(2)^2) - hk(2)*vk(1);
    B_hqf = [];
    B_hqin = [];
    if n_traj >= 3
        B_hqf = (3/(hk(end-1)*hk(end)))*((qk(end) - qk(end-1))*hk(end-1)^2 + (qk(end-1) - qk(end-2))*hk(end-1)^2) - hk(end-1)*vk(end);
        for k = 2:n_traj-2
            B_hqin = [B_hqin; (3/(hk(k)*hk(k+1)))*((qk(k+2) - qk(k+1))*hk(k)^2 + (qk(k+1) - qk(k))*hk(k+1)^2)];
        end
    end
    B_hq = [B_hq1; B_hqin; B_hqf];
    v_mid = inv(A_h)*B_hq;
    
    for k = 2:n_traj
        a23 = subs(a23, {sym(strcat("v", num2str(k)))}, {v_mid(k-1)});
    end
    for k = 1:n_traj
        th = subs(th, {strcat("a", num2str(k), "2"), ...
                       strcat("a", num2str(k), "3")}, ...
                       {a23(1, k), a23(2, k)});
        thd = subs(thd, {strcat("a", num2str(k), "2"), ...
                       strcat("a", num2str(k), "3")}, ...
                       {a23(1, k), a23(2, k)});
        thdd = subs(thdd, {strcat("a", num2str(k), "2"), ...
                       strcat("a", num2str(k), "3")}, ...
                       {a23(1, k), a23(2, k)});
    end
    for k = 2:n_traj
        th = subs(th, {strcat("v", num2str(k))}, ...
                      {v_mid(k-1)});
        thd = subs(thd, {strcat("v", num2str(k))}, ...
                      {v_mid(k-1)});
        thdd = subs(thdd, {strcat("v", num2str(k))}, ...
                      {v_mid(k-1)});
    end

end

