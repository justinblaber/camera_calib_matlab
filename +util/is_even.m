function success = is_even(num)
    % Determines if input is a real, finite, and even number.
    % 
    % Inputs:
    %   num - scalar; 
    %
    % Outputs:
    %   success - logical;
    
    success = util.is_int(num) && mod(num,2) == 0;
end