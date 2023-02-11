function plot_robot_pose(joint_types, DH_table, O_A_i, show_rotation)
    % plot_robot_pose - Plot the robot pose of a given DH table and
    % matrices
    %
    % plot_robot_pose(joint_types, DH_table, O_A_i)
    %
    % input:
    %   joint_types - Joint types: 
    %                           "r" revolute; 
    %                           "p" prismatic;
    %                           "ee" end effector
    %   DH_table - Table with the DH parameters for each joint
    %   O_A_i - Homogeneous transformation DH matrices from the origin to
    %   the i-th joint. Is an 4x(n*4) matrix where n is the number of
    %   joints and the (i*4):(i+1)*4-1 column is the matrix for the i-th
    %   joint
   
    % Joint positions from the DH matrix O_A_i
    joint_pos = [];
    % The axis of the joints' reference frame
    joint_x = [];
    joint_y = [];
    joint_z = [];
    % DH table values
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
    comm_norm_color = [1 0.7 0.2];
    joint_color = [.9 .9 .9];

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

        for j = 1:size(joint_pos, 2)
            from = joint_pos(:, j);
            to_x = joint_x(:, j);
            to_y = joint_y(:, j);
            to_z = joint_z(:, j);

            % Plot The frame with the x, y and z text
            quiver3(from(1), from(2), from(3), to_x(1), to_x(2), to_x(3), Color = x_color, LineWidth=line_width)
            quiver3(from(1), from(2), from(3), to_y(1), to_y(2), to_y(3), Color = y_color, LineWidth=line_width)
            quiver3(from(1), from(2), from(3), to_z(1), to_z(2), to_z(3), Color = z_color, LineWidth=line_width)
            text(from(1) + to_x(1), from(2) + to_x(2), from(3) + to_x(3), ...
                 strcat("x", num2str(j-1)), Color = x_color, FontSize=font_size, ...
                 HorizontalAlignment="center", VerticalAlignment="middle",FontWeight="bold")
            text(from(1) + to_y(1), from(2) + to_y(2), from(3) + to_y(3), ...
                 strcat("y", num2str(j-1)), Color = y_color, FontSize=font_size, ...
                 HorizontalAlignment="center", VerticalAlignment="middle",FontWeight="bold")
            text(from(1) + to_z(1), from(2) + to_z(2), from(3) + to_z(3), ...
                 strcat("z", num2str(j-1)), Color = z_color, FontSize=font_size, ...
                 HorizontalAlignment="center", VerticalAlignment="middle",FontWeight="bold")
            
            % From the second joint on I have to add x_j axis to the 
            % reference frame RF_j-1 in order to do the theta_j rotation 
            % and also the z_j-1 axis to the reference frame RF_ in order 
            % to do the alpha_j rotation
            if j > 1
                % k = j-1
                
                % Handle Theta_j
                % This new axis go from the origin of the previous RF in
                % the direction of this axis in current RF
                from_x_k = joint_pos(:, j-1);
                % If the angle theta_j is 0 then the x_j-1 axis overlap
                % x_j axis then the offset for the text is setted
                offset = 0;
                if DH_th(j-1) == 0
                    offset = 0.05;
                end
                % The x_j axis on the RF_j-1 with text
                quiver3(from_x_k(1), from_x_k(2), from_x_k(3), to_x(1), to_x(2), to_x(3), Color = x_color, LineStyle="--", LineWidth=line_width)
                text(from_x_k(1) + to_x(1), from_x_k(2) + to_x(2), from_x_k(3) + to_x(3)-offset, ...
                     strcat("x", num2str(j-1)), Color = x_color, FontSize=font_size, ...
                     HorizontalAlignment="center", VerticalAlignment="middle",FontWeight="bold")
                if show_rotation
                    % Semicircle for highlight theta_j
                    t = linspace(0,DH_th(j-1), 20);
                    v_th = [];
                    f_th = [];
                    v_line_th = [];
                    % Create a shape and a line for a semicircle of angle theta_j
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
                    % Estrapolate the rotation and position of the reference 
                    % frame RF_j-1 of the joint wrt the world frame
                    rot = O_A_i(1:3, (j-2)*4 + 1:(j-2)*4 + 3);
                    tran = O_A_i(1:3, (j-2)*4 + 4);
                    % Rotate and translate the semicircle
                    v_th = double(vpa(simplify(rot*v_th' + tran), 4));
                    v_th = v_th';
                    v_line_th = double(vpa(simplify(rot*v_line_th' + tran), 4));
                    v_line_th = v_line_th';
                    % Plot the semicircle shape with
                    patch(Faces = f_th, Vertices = v_th, FaceColor = x_color, EdgeColor='none', FaceAlpha=face_alpha/2);
                    % Plot the text of theta_j with its value in the semicircle
                    text((joint_pos(1, j-1) + 0.7*joint_x(1, j-1) + from_x_k(1) + 0.7*to_x(1))/2, ...
                         (joint_pos(2, j-1) + 0.7*joint_x(2, j-1) + from_x_k(2) + 0.7*to_x(2))/2, ...
                         (joint_pos(3, j-1) + 0.7*joint_x(3, j-1) + from_x_k(3) + 0.7*to_x(3))/2, ...
                         strcat("Θ", num2str(j-1), "=", num2str(double(DH_th(j-1)))), Color=x_color, FontSize=font_size, ...
                         HorizontalAlignment="center", VerticalAlignment="middle",FontWeight="bold")
                    % Plot the line on the border of the semicircle
                    line(v_line_th(1:end-1,1), v_line_th(1:end-1,2), v_line_th(1:end-1,3), ...
                          Color = x_color, LineWidth=line_width, LineStyle="--")
                    % Plot the head of the arrow witha cone
                    from = v_line_th(end-1,:);
                    to = v_line_th(end,:);
                    [x_c, y_c, z_c] = cylinder2P([0.03 0], 7, from, to);
                    x_c = double(x_c);
                    y_c = double(y_c);
                    z_c = double(z_c);
                    surface(x_c, y_c, z_c, FaceColor=x_color, EdgeColor='none')
                end


                % Handle Alpha_j
                % This new axis go from the origin of the current RF in
                % the direction of this axis in the previous RF
                from_z_j = joint_pos(:, j);
                to_z_k = joint_z(:, j-1);
                % If the angle alpha_j is 0 then the z_j-1 axis overlap
                % z_j axis then the offset for the text is setted
                offset = 0;
                if DH_al(j-1) == 0
                    offset = 0.05;
                end
                % The z_j-1 axis on the RF_j with text
                quiver3(from_z_j(1), from_z_j(2), from_z_j(3), to_z_k(1), to_z_k(2), to_z_k(3), Color = z_color, LineStyle="--", LineWidth=line_width)
                text(from_z_j(1) + to_z_k(1), from_z_j(2) + to_z_k(2), from_z_j(3) + to_z_k(3)-offset, ...
                     strcat("z", num2str(j-2)), Color = z_color, FontSize=font_size,...
                     HorizontalAlignment="center", VerticalAlignment="middle",FontWeight="bold")
                
                if show_rotation
                    % Semicircle for highlight alpha_j
                    t = linspace(0,DH_al(j-1), 20);
                    v_al = [];
                    f_al = [];
                    v_line_al = [];
                    % Create a shape and a line for a semicircle of angle alpha_j
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
                    % Estrapolate the rotation and position of the reference 
                    % frame RF_j of the joint wrt the world frame
                    rot = O_A_i(1:3, (j-1)*4 + 1:(j-1)*4 + 3);
                    tran = O_A_i(1:3, (j-1)*4 + 4);
                    % Rotate and translate the semicircle
                    v_al = double(vpa(simplify(rot*v_al' + tran), 4));
                    v_al = v_al';
                    v_line_al = double(vpa(simplify(rot*v_line_al' + tran), 4));
                    v_line_al = v_line_al';
                    % Plot the semicircle shape with
                    patch(Faces = f_al, Vertices = v_al, FaceColor = z_color, EdgeColor='none', FaceAlpha=face_alpha/2);
                    % Plot the text of alpha_j with its value in the semicircle
                    text((joint_pos(1, j) + 0.7*joint_z(1, j) + joint_pos(1, j) + 0.7*to_z_k(1))/2, ...
                         (joint_pos(2, j) + 0.7*joint_z(2, j) + joint_pos(2, j) + 0.7*to_z_k(2))/2, ...
                         (joint_pos(3, j) + 0.7*joint_z(3, j) + joint_pos(3, j) + 0.7*to_z_k(3))/2, ...
                         strcat("α", num2str(j-1), "=", num2str(double(DH_al(j-1)))), Color=z_color, FontSize=font_size, ...
                         HorizontalAlignment="center", VerticalAlignment="middle",FontWeight="bold")
                    % Plot the line on the border of the semicircle
                    line(v_line_al(2:end,1), v_line_al(2:end,2), v_line_al(2:end,3), ...
                          Color = z_color, LineWidth=line_width, LineStyle="--")
                    % Plot the head of the arrow witha cone
                    from = v_line_al(2,:);
                    to = v_line_al(1,:);
                    [x_c, y_c, z_c] = cylinder2P([0.03 0], 7, from, to);
                    x_c = double(x_c);
                    y_c = double(y_c);
                    z_c = double(z_c);
                    surface(x_c, y_c, z_c, FaceColor=z_color, EdgeColor='none')
                end
            end
            % JOINT PART
            if joint_types(j) == "ee"
                % In case of the end effector
                from1 = joint_pos(:, j) - r_revolute*joint_y(:, j)*1.5;
                to1 = joint_pos(:, j) + r_revolute*joint_y(:, j)*1.5;
                radii = radii_ee;
                [x_c, y_c, z_c] = cylinder2P(radii, 20, from1', to1');
                x_c = double(x_c);
                y_c = double(y_c);
                z_c = double(z_c);
                surface(x_c, y_c, z_c, FaceColor=joint_color, EdgeColor='none', FaceAlpha=face_alpha)

                from2 = from1;
                to2 = from1 + r_revolute*joint_z(:, j)*1.5;
                [x_c, y_c, z_c] = cylinder2P(radii, 20, from2', to2');
                x_c = double(x_c);
                y_c = double(y_c);
                z_c = double(z_c);
                surface(x_c, y_c, z_c, FaceColor=[.9 .9 .9], EdgeColor='none', FaceAlpha=face_alpha)

                from3 = to1;
                to3 = to1 + r_revolute*joint_z(:, j)*1.5;
                [x_c, y_c, z_c] = cylinder2P(radii, 20, from3', to3');
                x_c = double(x_c);
                y_c = double(y_c);
                z_c = double(z_c);
                surface(x_c, y_c, z_c, FaceColor=joint_color, EdgeColor='none', FaceAlpha=face_alpha)
            else
                % The revolute joint and the base of a prismatic joint
                from = joint_pos(:, j) - r_revolute*joint_z(:, j);
                to = joint_pos(:, j) + r_revolute*joint_z(:, j);
                radii = radii_revolute;
                [x_c, y_c, z_c] = cylinder2P(radii, 20, from', to');
                x_c = double(x_c);
                y_c = double(y_c);
                z_c = double(z_c);
                surface(x_c, y_c, z_c, FaceColor=joint_color, EdgeColor='none', FaceAlpha=face_alpha)
    
                if joint_types(j) == "p"
                    % The second part of a prismatic joint
                    from = joint_pos(:, j) - r_revolute*joint_z(:, j)*1.5;
                    to = joint_pos(:, j) + r_revolute*joint_z(:, j)*1.5;
                    radii = radii_prismatic;
                    [x_c, y_c, z_c] = cylinder2P(radii, 7, from', to');
                    x_c = double(x_c);
                    y_c = double(y_c);
                    z_c = double(z_c);
                    surface(x_c, y_c, z_c, FaceColor=joint_color, EdgeColor='none', FaceAlpha=face_alpha)
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