function pos_def = is_pos_def(A)
    % Checks if input array is positive definite
    %
    % Inputs:
    %   A - array; MxM array
    % 
    % Outputs:
    %   pos_def - logical; true if input array is positive definite
    
    % Initialize to false
    pos_def = false;
    
    % Do cholesky factorization
    [~, p] = chol(A);
    if p == 0
        pos_def = true;
    end
end