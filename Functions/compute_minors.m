function minors = compute_minors(A)
    % compute_minors - Compute all the minors of a matrix A 
    % minors = compute_minors(A)
    %
    % input:
    %   A - a matrix 
    %
    % output:
    %   minors - a matrix containing all the minors of A

    
    % Calculate the size of matrix A
    [n, m] = size(A);
    
    % Initialize a matrix of zeros to hold the minors
    minors = sym(zeros(n,m));
    
    % Calculate the determinant of all 2x2 submatrices of A
    for i = 1:n
        for j = 1:m
            % Extract the 2x2 submatrix starting from the current position
            submatrix = A([1:i-1 i+1:n], [1:j-1 j+1:m]);
            % Calculate the determinant of the 2x2 submatrix, only if the
            % submatrix is 2x2 or larger
            if size(submatrix,1) >= 2 && size(submatrix,2) >= 2
                minors(i,j) = simplify(det(submatrix(1:2,1:2)));
            end
        end
    end
end

