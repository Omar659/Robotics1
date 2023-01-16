function check = check_matrix(A)
    % check orthogonality
    check_orth = true;
    for i = 1:length(A)
        for j = i+1:length(A)
            value = dot(A(:,i),A(:,j));
            if value~=0
                check_orth=false;
                break;
            end
        end 
    end 
    % check normality
    check_n = true;
    for i=1:length(A)
        n = norm(A(:, i)) == 1;
        if ~n
            check_n = false;
            break
        end
    end
    % check determinant
    check_det = abs(det(A) - 1) < 1e-6;
    % combine checks
    check = check_orth && check_n && check_det;
end