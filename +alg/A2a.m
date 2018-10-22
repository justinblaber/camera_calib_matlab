function a = A2a(A)
    % Converts camera matrix to camera vector.
    %
    % Inputs:
    %   A - array; 3x3 camera matrix
    % 
    % Outputs:
    %   a - array; 3x1 array containing:
    %       [alpha; x_o; y_o]

    a = A([1 7 8])';
end