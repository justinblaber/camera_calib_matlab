function success = is_even(array)
    % Determines if input is a real, finite, and even
    %
    % Inputs:
    %   array - array;
    %
    % Outputs:
    %   success - logical;

    success = alg.is_int(array) & mod(array, 2) == 0;
end
