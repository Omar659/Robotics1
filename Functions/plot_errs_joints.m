function plot_errs_joints(history, speed, t, ylab, fig, q_des)
    % rotation_around_r - Compute the rotation matrix around a fixed
    % unitary vector r
    %
    % Rtr = rotation_around_r(r_m, theta)
    %
    % input:
    %   r_m - A 3x1 vector along which we need to rotate
    %   theta - The angle of rotation
    %
    % output:
    %   Rtr - The 3x3 rotation matrix around r_m of angle theta
    if nargin < 6
        q_des = 0;
    end
    

    figure(fig)
    x = [];
    y = [];
    plot(x,y)
%     if 0 ~= q_des 
%         hold on;
%         plot(x1, y1, 'r-', 'LineWidth', 2);
%         legend('red line','Location','northwest');
%     end
    xlabel('Iterations')
    ylabel(ylab)
    title(t)
    grid on
    for i = 1: length(history)
        x = [x i];
        y = [y history(i)];
        plot(x,y)
%         if 0 ~= q_des 
%             hold on;
%             plot(x1, y1, 'r-', 'LineWidth', 2);
%             legend('red line','Location','northwest');
%         end
        xlabel('Iterations')
        ylabel(ylab)
        title(t)
        grid on
        drawnow %aggiorna il grafico in modo dinamico
        pause(speed) %aspetta 0.1 secondi prima di aggiungere un nuovo punto
    end

end