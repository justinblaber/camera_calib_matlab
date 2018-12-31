function success = is_pos_def(A)
    % Checks if input array is positive definite
    %
    % Inputs:
    %   A - array; MxM array
    %
    % Outputs:
    %   success - logical; true if input array is positive definite

    % Initialize to false
    success = false;

    if ~isempty(A)
        % Do cholesky factorization
        [~, p] = chol(A);
        if p == 0
            success = true;
        end
    end
end
