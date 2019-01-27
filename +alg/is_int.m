function success = is_int(array)
    % Determines if input is a real, finite, and integer
    %
    % Inputs:
    %   array - array;
    %
    % Outputs:
    %   success - logical;

    success = alg.is_real_and_finite(array) & round(array) == array;
end
