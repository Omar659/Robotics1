clear all
clc

addpath("Functions\")

% general rotation around r
%R = rotation_r([sym('rx') sym('ry') sym('rz')], sym('t'));

disp("Rotation around x-axis of -pi")
R = rotation_around_r([1 0 0], -pi);
disp(R)

disp("Compute the angle theta of the matrix R")
[t1, t2, t3] = compute_theta(R);
disp("First tecnique with arcos(trace(R)/2)")
disp(t1)
disp("Second tecnique with atan2(+ (1/2)*norm(r), trace(R)/2)")
disp(t2)
disp("Second tecnique with atan2(- (1/2)*norm(r), trace(R)/2)")
disp(t3)

disp("Comput r given the matrix R and the angle theta")
disp("NB. the output r can be 1 value, 2 value or 'no solution'")
r = compute_r(R, t2);
disp(r)

exercise_slide7_pag24()

function exercise_slide7_pag24()
    disp("Homework: write a code that determines the two solutions (theta, r)")
    R  = [-1 0 0; 
          0 -(1/sqrt(2)) -(1/sqrt(2));
          0 -(1/sqrt(2)) (1/sqrt(2))];
    disp('for R = ')
    disp(R)
    check = check_matrix(R);
    if ~check
        disp("The matrix is not normal or not ortogonal or with a determinat different from +1")
    else
        disp("The matrix is ok")
        disp('Solution:')
        [t1, t2, t3] = compute_theta(R);
        disp('theta values are:')
        disp(strcat('with arcos -> ',num2str(t1)))
        disp(strcat('with atan2 positive sin -> ',num2str(t2)))
        disp(strcat('with atan2 negative sin -> ',num2str(t3)))
        disp('r values with atan2 results:')
        r = compute_r(R, t2);
        disp('r with atan2:')
        disp(r)
        disp('Check results')
        Rtr1 = rotation_around_r(r(:,1), t2);
        Rtr2 = rotation_around_r(r(:,2), t3);
        disp('R with atan2 positive sign:')
        disp(vpa(Rtr1, 4))
        disp('R with atan2 negative sign:')
        disp(vpa(Rtr2, 4))
    end
end