function [sol1, sol2] = angles_from_RPY(R)
    % angles_from_RPY - return the angle theta, phi and psi from a
    % Roll-Pitch-Yaw rotation matrix R
    %
    % sintax: [sol1, sol2] = angles_from_RPY(R)
    %
    % input:
    %   R - Roll-Pitch-Yaw rotation matrix R
    %
    % output:
    %   [sol1, sol2] - The two solutione since the atan2 function of theta
    %   take as cos a plus/manus value, resulting in two output
    theta1 = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
    theta2 = atan2(-R(3,1), -sqrt(R(3,2)^2 + R(3,3)^2));
    phi1 = atan2(R(2,1)/cos(theta1), R(1,1)/cos(theta1));
    phi2 = atan2(R(2,1)/cos(theta2), R(1,1)/cos(theta2));
    psi1 = atan2(R(3,2)/cos(theta1), R(3,3)/cos(theta1));
    psi2 = atan2(R(3,2)/cos(theta2), R(3,3)/cos(theta2));
    sol1 = [theta1; phi1; psi1];
    sol2 = [theta2; phi2; psi2];
    disp("Angles order: theta, phi, psi")
end