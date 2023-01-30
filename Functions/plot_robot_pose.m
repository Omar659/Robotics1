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

    % Colors
    x_color = [.7 0 0];
    y_color = [0 .7 0];
    z_color = [0 0 .7];
    disp_color = [0 0.5 1];
    comm_norm_color = [1 0.9 0.3];

    % Plot parameters
    line_width = 2;
    font_size = 10;
    
    % Radii of the cylinders
    radii_normal = ones(1, 22)*r_finish;
    radii_normal = [0 radii_normal 0];
    radii_prismatic_link = linspace(r_start, r_finish, 24);
    radii_prismatic_link = [0 radii_prismatic_link(1:6) ...
                            radii_prismatic_link(7:12)*0.9  ...
                            radii_prismatic_link(13:18)*0.83  ...
                            radii_prismatic_link(19:24)*0.77 0];
    radii_ee = ones(1,22)*r_revolute*0.3;
    radii_ee = [0 radii_ee 0];
    radii_revolute = ones(1,22)*r_revolute;
    radii_revolute = [0 radii_revolute 0];
    radii_prismatic = ones(1,22)*r_revolute*0.9;
    radii_prismatic = [0 radii_prismatic 0];

    view(3);
    hold on
        % Joints plot
        for i = 2:size(joint_pos, 2)
            % Radii for prismatic/revolute joint
            radii = radii_normal;
            if joint_types(i-1) == "p"
                radii = radii_prismatic_link;
            end

            % Displacement
            to = joint_pos(:, i-1) + DH_d(i-1)*joint_z(:, i-1);
            [x_c, y_c, z_c] = cylinder2P(radii, 20, joint_pos(:, i-1)', to');
            x_c = double(x_c);
            y_c = double(y_c);
            z_c = double(z_c);            
            surface(x_c, y_c, z_c, FaceColor=disp_color, EdgeColor='none', FaceAlpha=face_alpha)
            disp_from = joint_pos(:, i-1) + 1.3*r_revolute*joint_x(:, i-1);
            disp_to = joint_pos(:, i-1) + DH_d(i-1)*joint_z(:, i-1) + 1.3*r_revolute*joint_x(:, i-1);
            line([disp_from(1) disp_to(1)], [disp_from(2) disp_to(2)], [disp_from(3) disp_to(3)], ...
                  Color = disp_color, LineWidth=line_width)
            text((disp_from(1) + disp_to(1))/2, (disp_from(2) + disp_to(2))/2, (disp_from(3) + disp_to(3))/2, ...
                 strcat("d", num2str(i-1), "=", num2str(double(DH_d(i-1)))), ...
                 Color = disp_color, FontSize=font_size, ...
                 HorizontalAlignment="right", VerticalAlignment="middle",FontWeight="bold")

            % Common normal
            radii = radii_normal;
            to = joint_pos(:, i) - DH_a(i-1)*joint_x(:, i);
            [x_c, y_c, z_c] = cylinder2P(radii, 20, joint_pos(:, i)', to');
            x_c = double(x_c);
            y_c = double(y_c);
            z_c = double(z_c);
            surface(x_c, y_c, z_c, FaceColor=comm_norm_color, EdgeColor='none', FaceAlpha=face_alpha)
            disp_from = joint_pos(:, i) + 1.3*r_revolute*joint_z(:, i);
            disp_to = joint_pos(:, i) - DH_a(i-1)*joint_x(:, i) + 1.3*r_revolute*joint_z(:, i);
            line([disp_from(1) disp_to(1)], [disp_from(2) disp_to(2)], [disp_from(3) disp_to(3)], ...
                  Color = comm_norm_color, LineWidth=line_width)
            text((disp_from(1) + disp_to(1))/2, (disp_from(2) + disp_to(2))/2, (disp_from(3) + disp_to(3))/2, ...
                 strcat("a", num2str(i-1), "=", num2str(double(DH_a(i-1)))), ...
                 Color = comm_norm_color, FontSize=font_size, ...
                 HorizontalAlignment="right", VerticalAlignment="middle",FontWeight="bold")
        end
        for p = 1:size(joint_pos, 2)
            from = joint_pos(:, p);
            to_x = joint_x(:, p);
            to_y = joint_y(:, p);
            to_z = joint_z(:, p);
            quiver3(from(1), from(2), from(3), to_x(1), to_x(2), to_x(3), Color = x_color, LineWidth=line_width)
            quiver3(from(1), from(2), from(3), to_y(1), to_y(2), to_y(3), Color = y_color, LineWidth=line_width)
            quiver3(from(1), from(2), from(3), to_z(1), to_z(2), to_z(3), Color = z_color, LineWidth=line_width)
            text(from(1) + to_x(1), from(2) + to_x(2), from(3) + to_x(3), ...
                 strcat("x", num2str(p-1)), Color = x_color, FontSize=font_size, ...
                 HorizontalAlignment="center", VerticalAlignment="middle",FontWeight="bold")
            text(from(1) + to_y(1), from(2) + to_y(2), from(3) + to_y(3), ...
                 strcat("y", num2str(p-1)), Color = y_color, FontSize=font_size, ...
                 HorizontalAlignment="center", VerticalAlignment="middle",FontWeight="bold")
            text(from(1) + to_z(1), from(2) + to_z(2), from(3) + to_z(3), ...
                 strcat("z", num2str(p-1)), Color = z_color, FontSize=font_size, ...
                 HorizontalAlignment="center", VerticalAlignment="middle",FontWeight="bold")
            
            if p > 1
                from_x_j = joint_pos(:, p-1);
                offset = 0;
                if DH_th(p-1) == 0
                    offset = 0.05;
                end
                quiver3(from_x_j(1), from_x_j(2), from_x_j(3), to_x(1), to_x(2), to_x(3), Color = x_color, LineStyle="--", LineWidth=line_width)
                text(from_x_j(1) + to_x(1), from_x_j(2) + to_x(2), from_x_j(3) + to_x(3)-offset, ...
                     strcat("x", num2str(p-1)), Color = x_color, FontSize=font_size, ...
                     HorizontalAlignment="center", VerticalAlignment="middle",FontWeight="bold")
                from_z_j = joint_pos(:, p);
                to_z_j = joint_z(:, p-1);
                offset = 0;
                if DH_al(p-1) == 0
                    offset = 0.05;
                end
                quiver3(from_z_j(1), from_z_j(2), from_z_j(3), to_z_j(1), to_z_j(2), to_z_j(3), Color = z_color, LineStyle="--", LineWidth=line_width)
                text(from_z_j(1) + to_z_j(1), from_z_j(2) + to_z_j(2), from_z_j(3) + to_z_j(3)-offset, ...
                     strcat("z", num2str(p-2)), Color = z_color, FontSize=font_size,...
                     HorizontalAlignment="center", VerticalAlignment="middle",FontWeight="bold")

                % Circle for theta
                t = linspace(0,DH_th(p-1), 20);
                v_th = [];
                f_th = [];
                v_line_th = [];
                for i = 1:length(t)-1
                    v1 = 0.7*[0 0 0];
                    v2 = 0.7*[cos(t(i)) sin(t(i)) 0*t(i)];
                    v3 = 0.7*[cos(t(i+1)) sin(t(i+1)) 0*t(i+1)];
                    v_th = [v_th; v1; v2; v3];
                    v_line_new = 0.7*[cos(t(i)) sin(t(i)) 0*t(i)];
                    v_line_th = [v_line_th; v_line_new];
                    f_i = [((i-1)*3)+1 ((i-1)*3)+2 ((i-1)*3)+3];
                    f_th = [f_th; f_i];
                end
                v_line_new = 0.7*[cos(t(end)) sin(t(end)) 0*t(end)];
                v_line_th = [v_line_th; v_line_new];
                
                rot = O_A_i(1:3, (p-2)*4 + 1:(p-2)*4 + 3);
                tran = O_A_i(1:3, (p-2)*4 + 4);
                v_th = double(vpa(simplify(rot*v_th' + tran), 4));
                v_th = v_th';
                v_line_th = double(vpa(simplify(rot*v_line_th' + tran), 4));
                v_line_th = v_line_th';
                patch(Faces = f_th, Vertices = v_th, FaceColor = x_color, EdgeColor='none', FaceAlpha=face_alpha/2);
                text((joint_pos(1, p-1) + 0.7*joint_x(1, p-1) + from_x_j(1) + 0.7*to_x(1))/2, ...
                     (joint_pos(2, p-1) + 0.7*joint_x(2, p-1) + from_x_j(2) + 0.7*to_x(2))/2, ...
                     (joint_pos(3, p-1) + 0.7*joint_x(3, p-1) + from_x_j(3) + 0.7*to_x(3))/2, ...
                     strcat("Θ", num2str(p-1), "=", num2str(double(DH_th(p-1)))), Color=x_color, FontSize=font_size, ...
                     HorizontalAlignment="center", VerticalAlignment="middle",FontWeight="bold")
                line(v_line_th(1:end-1,1), v_line_th(1:end-1,2), v_line_th(1:end-1,3), ...
                      Color = x_color, LineWidth=line_width, LineStyle="--")
                from = v_line_th(end-1,:);
                to = v_line_th(end,:);
                [x_c, y_c, z_c] = cylinder2P([0.03 0], 7, from, to);
                x_c = double(x_c);
                y_c = double(y_c);
                z_c = double(z_c);
                surface(x_c, y_c, z_c, FaceColor=x_color, EdgeColor='none')

                
                % Circle for alpha
                t = linspace(0,DH_al(p-1), 20);
                v_al = [];
                f_al = [];
                v_line_al = [];
                for i = 1:length(t)-1
                    v1 = 0.7*[0 0 0];
                    v2 = 0.7*[0*t(i) sin(t(i)) cos(t(i))];
                    v3 = 0.7*[0*t(i+1) sin(t(i+1)) cos(t(i+1))];
                    v_al = [v_al; v1; v2; v3];
                    v_line_new = 0.7*[0*t(i) sin(t(i)) cos(t(i))];
                    v_line_al = [v_line_al; v_line_new];
                    f_i = [((i-1)*3)+1 ((i-1)*3)+2 ((i-1)*3)+3];
                    f_al = [f_al; f_i];
                end
                v_line_new = 0.7*[0*t(end) sin(t(end)) cos(t(end))];
                v_line_al = [v_line_al; v_line_new];

                rot = O_A_i(1:3, (p-1)*4 + 1:(p-1)*4 + 3);
                tran = O_A_i(1:3, (p-1)*4 + 4);
                v_al = double(vpa(simplify(rot*v_al' + tran), 4));
                v_al = v_al';
                v_line_al = double(vpa(simplify(rot*v_line_al' + tran), 4));
                v_line_al = v_line_al';
                patch(Faces = f_al, Vertices = v_al, FaceColor = z_color, EdgeColor='none', FaceAlpha=face_alpha/2);
                text((joint_pos(1, p) + 0.7*joint_z(1, p) + joint_pos(1, p) + 0.7*to_z_j(1))/2, ...
                     (joint_pos(2, p) + 0.7*joint_z(2, p) + joint_pos(2, p) + 0.7*to_z_j(2))/2, ...
                     (joint_pos(3, p) + 0.7*joint_z(3, p) + joint_pos(3, p) + 0.7*to_z_j(3))/2, ...
                     strcat("α", num2str(p-1), "=", num2str(double(DH_al(p-1)))), Color=z_color, FontSize=font_size, ...
                     HorizontalAlignment="center", VerticalAlignment="middle",FontWeight="bold")
                line(v_line_al(2:end,1), v_line_al(2:end,2), v_line_al(2:end,3), ...
                      Color = z_color, LineWidth=line_width, LineStyle="--")
                from = v_line_al(2,:);
                to = v_line_al(1,:);
                [x_c, y_c, z_c] = cylinder2P([0.03 0], 7, from, to);
                x_c = double(x_c);
                y_c = double(y_c);
                z_c = double(z_c);
                surface(x_c, y_c, z_c, FaceColor=z_color, EdgeColor='none')
            end
            if joint_types(p) == "ee"
                from1 = joint_pos(:, p) - r_revolute*joint_y(:, p)*1.5;
                to1 = joint_pos(:, p) + r_revolute*joint_y(:, p)*1.5;
                radii = radii_ee;
                [x_c, y_c, z_c] = cylinder2P(radii, 20, from1', to1');
                x_c = double(x_c);
                y_c = double(y_c);
                z_c = double(z_c);
                surface(x_c, y_c, z_c, FaceColor=[.9 .9 .9], EdgeColor='none', FaceAlpha=face_alpha)

                from2 = from1;
                to2 = from1 + r_revolute*joint_z(:, p)*1.5;
                [x_c, y_c, z_c] = cylinder2P(radii, 20, from2', to2');
                x_c = double(x_c);
                y_c = double(y_c);
                z_c = double(z_c);
                surface(x_c, y_c, z_c, FaceColor=[.9 .9 .9], EdgeColor='none', FaceAlpha=face_alpha)

                from3 = to1;
                to3 = to1 + r_revolute*joint_z(:, p)*1.5;
                [x_c, y_c, z_c] = cylinder2P(radii, 20, from3', to3');
                x_c = double(x_c);
                y_c = double(y_c);
                z_c = double(z_c);
                surface(x_c, y_c, z_c, FaceColor=[.9 .9 .9], EdgeColor='none', FaceAlpha=face_alpha)
            else
                from = joint_pos(:, p) - r_revolute*joint_z(:, p);
                to = joint_pos(:, p) + r_revolute*joint_z(:, p);
                radii = radii_revolute;
                [x_c, y_c, z_c] = cylinder2P(radii, 20, from', to');
                x_c = double(x_c);
                y_c = double(y_c);
                z_c = double(z_c);
                surface(x_c, y_c, z_c, FaceColor=[.9 .9 .9], EdgeColor='none', FaceAlpha=face_alpha)
    
                if joint_types(p) == "p"
                    from = joint_pos(:, p) - r_revolute*joint_z(:, p)*1.5;
                    to = joint_pos(:, p) + r_revolute*joint_z(:, p)*1.5;
                    radii = radii_prismatic;
                    [x_c, y_c, z_c] = cylinder2P(radii, 7, from', to');
                    x_c = double(x_c);
                    y_c = double(y_c);
                    z_c = double(z_c);
                    surface(x_c, y_c, z_c, FaceColor=[.9 .9 .9], EdgeColor='none', FaceAlpha=face_alpha)
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