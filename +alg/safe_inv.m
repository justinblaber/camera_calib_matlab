function Y = safe_inv(X)
    % Wrapper for inv to handle cases where inputs cause an error.
    %
    % Inputs:
    %   X - array; input to inv()
    %
    % Outputs:
    %   Y; array; output of inv()

    % Initialize
    safe = true;

    if ismatrix(X) && size(X, 1) == size(X, 2) && any(isnan(X(:)))
        safe = false;
    end

    % Call function if its safe
    if safe
        Y = inv(X);
    else
        % Output nans
        Y = nan(size(X));
    end
end
