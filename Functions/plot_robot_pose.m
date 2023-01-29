function plot_robot_pose(joint_types, DH_table, O_A_i)
    % compute_theta - Compute the theta angle of the rotation matrix R
    %
    % [t1, t2, t3] = compute_theta(R)
    %
    % input:
    %   R - A 3x3 rotation matrix
    %
    % output:
    %   t1 - Return the theta angle calculated with the acos function (not
    %   optimal)
    %   t2 - Return the theta angle calculated with atan2 function with a
    %   positive square root as sine
    %   t2 - Return the theta angle calculated with atan2 function with a
    %   negative square root as sine
    

    joint_pos = [];
    joint_x = [];
    joint_y = [];
    joint_z = [];
    DH_a = [];
    DH_d = [];
    DH_th = [];
    DH_al = [];
    for i = 1:size(O_A_i, 2)/4
        joint_pos = [joint_pos O_A_i(1:3, i*4)];
        joint_x = [joint_x O_A_i(1:3, (i*4)-3)];
        joint_y = [joint_y O_A_i(1:3, (i*4)-2)];
        joint_z = [joint_z O_A_i(1:3, (i*4)-1)];
        if i < size(O_A_i, 2)/4
            DH_a = [DH_a DH_table(i, 1)];
            DH_al = [DH_al DH_table(i, 2)];
            DH_d = [DH_d DH_table(i, 3)];
            DH_th = [DH_th DH_table(i, 4)];
        end
    end

    % Revolute joint radius
    r_revolute = 0.05 + abs(max(mean(joint_pos, 2)))*0.15;

    % Cylinder radius
    r_start = r_revolute*0.8;
    r_finish = r_start*0.8;
    
    % Alpha channel
    face_alpha = 0.5;

    t = linspace(0,1/2*pi);
    v = [];
    f = [];
    for i = 1:length(t)-1
        v1 = 0.7*[0 0 0];
        v2 = 0.7*[cos(t(i)) sin(t(i)) 0*t(i)];
        v3 = 0.7*[cos(t(i+1)) sin(t(i+1)) 0*t(i+1)];
        v = [v; v1; v2; v3];
        f_i = [((i-1)*3)+1 ((i-1)*3)+2 ((i-1)*3)+3];
        f = [f; f_i];
    end
    rot = O_A_i(1:3, end-3:end-1);
    tran = O_A_i(1:3, end);
    v = double(vpa(simplify(rot*v' + tran), 4));
    v = v';
    patch(Faces = f, Vertices = v, FaceColor = [1 0 0], EdgeColor='none', FaceAlpha=face_alpha);

    view(3);
    hold on
        for i = 2:size(joint_pos, 2)
            radiuses = ones(1, 22)*r_finish;
            radiuses = [0 radiuses 0];
            if joint_types(i-1) == "p"
                radiuses = linspace(r_start, r_finish, 24);
                radiuses = [0 radiuses(1:6) radiuses(7:12)*0.9 radiuses(13:18)*0.83 radiuses(19:24)*0.77 0];
            end
            to = joint_pos(:, i-1) + DH_d(i-1)*joint_z(:, i-1);
            [x_c, y_c, z_c] = cylinder2P(radiuses, 20, joint_pos(:, i-1)', to');
            x_c = double(x_c);
            y_c = double(y_c);
            z_c = double(z_c);
            surf(x_c, y_c, z_c, FaceColor=[0 0.5 1], EdgeColor='none', FaceAlpha=face_alpha)

            radiuses = ones(1, 22)*r_finish;
            radiuses = [0 radiuses 0];
            to = joint_pos(:, i) - DH_a(i-1)*joint_x(:, i);
            [x_c, y_c, z_c] = cylinder2P(radiuses, 20, joint_pos(:, i)', to');
            x_c = double(x_c);
            y_c = double(y_c);
            z_c = double(z_c);
            surf(x_c, y_c, z_c, FaceColor=[1 0.9 0.3], EdgeColor='none', FaceAlpha=face_alpha)
        end
        for p = 1:size(joint_pos, 2)
            from = joint_pos(:, p);
            to_x = joint_x(:, p);
            to_y = joint_y(:, p);
            to_z = joint_z(:, p);
            quiver3(from(1), from(2), from(3), to_x(1), to_x(2), to_x(3), Color = [1 0 0])
            quiver3(from(1), from(2), from(3), to_y(1), to_y(2), to_y(3), Color = [0 1 0])
            quiver3(from(1), from(2), from(3), to_z(1), to_z(2), to_z(3), Color = [0 0 1])
            text(from(1) + to_x(1), from(2) + to_x(2), from(3) + to_x(3), strcat("x", num2str(p-1)))
            text(from(1) + to_y(1), from(2) + to_y(2), from(3) + to_y(3), strcat("y", num2str(p-1)))
            text(from(1) + to_z(1), from(2) + to_z(2), from(3) + to_z(3), strcat("z", num2str(p-1)))
            
            if p > 1
                from = joint_pos(:, p-1);
                quiver3(from(1), from(2), from(3), to_x(1), to_x(2), to_x(3), Color = [1 0 0], LineStyle="--")
                text(from(1) + to_x(1), from(2) + to_x(2), from(3) + to_x(3)-0.05, strcat("x", num2str(p-1)))
                from = joint_pos(:, p);
                to_z = joint_z(:, p-1);
                quiver3(from(1), from(2), from(3), to_z(1), to_z(2), to_z(3), Color = [0 0 1], LineStyle="--")
                text(from(1) + to_z(1), from(2) + to_z(2), from(3) + to_z(3)-0.05, strcat("z", num2str(p-2)))
            end
            if joint_types(p) == "ee"
                from1 = joint_pos(:, p) - r_revolute*joint_y(:, p)*1.5;
                to1 = joint_pos(:, p) + r_revolute*joint_y(:, p)*1.5;
                radiuses = ones(1,22)*r_revolute*0.3;
                radiuses = [0 radiuses 0];
                [x_c, y_c, z_c] = cylinder2P(radiuses, 20, from1', to1');
                x_c = double(x_c);
                y_c = double(y_c);
                z_c = double(z_c);
                surf(x_c, y_c, z_c, FaceColor=[.9 .9 .9], EdgeColor='none', FaceAlpha=face_alpha)

                from2 = from1;
                to2 = from1 + r_revolute*joint_z(:, p)*1.5;
                radiuses = ones(1,22)*r_revolute*0.3;
                radiuses = [0 radiuses 0];
                [x_c, y_c, z_c] = cylinder2P(radiuses, 20, from2', to2');
                x_c = double(x_c);
                y_c = double(y_c);
                z_c = double(z_c);
                surf(x_c, y_c, z_c, FaceColor=[.9 .9 .9], EdgeColor='none', FaceAlpha=face_alpha)

                from3 = to1;
                to3 = to1 + r_revolute*joint_z(:, p)*1.5;
                radiuses = ones(1,22)*r_revolute*0.3;
                radiuses = [0 radiuses 0];
                [x_c, y_c, z_c] = cylinder2P(radiuses, 20, from3', to3');
                x_c = double(x_c);
                y_c = double(y_c);
                z_c = double(z_c);
                surf(x_c, y_c, z_c, FaceColor=[.9 .9 .9], EdgeColor='none', FaceAlpha=face_alpha)
            else
                from = joint_pos(:, p) - r_revolute*joint_z(:, p);
                to = joint_pos(:, p) + r_revolute*joint_z(:, p);
                radiuses = ones(1,22)*r_revolute;
                radiuses = [0 radiuses 0];
                [x_c, y_c, z_c] = cylinder2P(radiuses, 20, from', to');
                x_c = double(x_c);
                y_c = double(y_c);
                z_c = double(z_c);
                surf(x_c, y_c, z_c, FaceColor=[.9 .9 .9], EdgeColor='none', FaceAlpha=face_alpha)
    
                if joint_types(p) == "p"
                    from = joint_pos(:, p) - r_revolute*joint_z(:, p)*1.5;
                    to = joint_pos(:, p) + r_revolute*joint_z(:, p)*1.5;
                    radiuses = ones(1,22)*r_revolute*0.9;
                    radiuses = [0 radiuses 0];
                    [x_c, y_c, z_c] = cylinder2P(radiuses, 7, from', to');
                    x_c = double(x_c);
                    y_c = double(y_c);
                    z_c = double(z_c);
                    surf(x_c, y_c, z_c, FaceColor=[.9 .9 .9], EdgeColor='none', FaceAlpha=face_alpha)
                end
            end
            
        end
        xlabel('x')
        ylabel('y')
        zlabel('z')
        grid on
        light
        lighting gouraud
    hold off

end