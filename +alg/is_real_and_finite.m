function success = is_real_and_finite(array)
    % Determines if input is real and finite.
    %
    % Inputs:
    %   array - array;
    %
    % Outputs:
    %   success - logical;

    success = arrayfun(@isnumeric, array) & isreal(array) & isfinite(array);
end
