function plot_errs_joints(history, speed, t, ylab, q_des)
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
    if nargin < 5
        q_des = 0;
    end
    
    y = [];
    plot(y)
    xlabel('Iterations')
    ylabel(ylab)
    xlim([1 length(history)])
    ylim([(min(history) - abs(mean(history))*0.1)  (max(history) + abs(mean(history))*0.1)])
    title(t)
    grid on
    for i = 1: length(history)
        y = [y history(i)];
        if 0 ~= q_des 
            hold on
            plot(y', LineStyle="-", Color="b");
            for j = 1:size(q_des, 2)
                plot(q_des(1, j)*ones(length(history))', LineStyle="--", Color="r");
            end
            hold off
        else
            plot(y', Color="b")
        end
        xlabel('Iterations')
        ylabel(ylab)
        xlim([1 length(history)])
        ylim([(min(history) - abs(mean(history))*0.1)  (max(history) + abs(mean(history))*0.1)])
        title(t)
        grid on
        drawnow %aggiorna il grafico in modo dinamico
        pause(speed) %aspetta 0.1 secondi prima di aggiungere un nuovo punto
    end

end