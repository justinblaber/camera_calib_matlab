function success = is_pos(array)
    % Determines if input is real, finite, and positive
    %
    % Inputs:
    %   array - array;
    %
    % Outputs:
    %   success - logical;

    success = alg.is_real_and_finite(array) & array > 0;
end
