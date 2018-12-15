function A = a2A(a)
    % Converts camera vector to camera matrix.
    %
    % Inputs:
    %   a - array; 3x1 array containing:
    %       [alpha; x_o; y_o]
    % 
    % Outputs:
    %   A - array; 3x3 camera matrix
        
    A = [a(1) 0    a(2);
         0    a(1) a(3);
         0    0    1];
end