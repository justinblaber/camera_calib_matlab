function a = A2a(A)
    % Converts camera matrix to camera vector.
    %
    % Inputs:
    %   A - array; 3x3 camera matrix
    %
    % Outputs:
    %   a - array; 3x1 array containing:
    %       [alpha; x_o; y_o]

    % Note: isequaln will test for equality and return true even if there
    %   are NaNs
    if any(A([2 3 4 6]) ~= 0) || A(9) ~= 1 || ~isequaln(A(1), A(5))
        error('Invalid camera matrix for conversion to camera vector');
    end

    a = A([1 7 8])';
end
