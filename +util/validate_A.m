function validate_A(A)
    % Validates input camera matrix.
    %
    % Inputs:
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %
    % Outputs:
    %   none
    
    if A(1,1) ~= A(2,2) || A(1,2) ~= 0 || A(2,1) ~= 0
        error('Camera matrix must have single focal point without skew.')
    end
end
