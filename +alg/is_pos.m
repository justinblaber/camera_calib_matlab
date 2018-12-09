function success = is_pos(num)
    % Determines if input is a real, finite, and positive number.
    % 
    % Inputs:
    %   num - scalar;
    %
    % Outputs:
    %   success - logical;
    
    success = alg.is_num(num) && num > 0;
end
