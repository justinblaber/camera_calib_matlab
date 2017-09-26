function success = is_num(num)
    % Determines if input is a real and finite number.
    % 
    % Inputs:
    %   num - scalar; 
    %
    % Outputs:
    %   success - logical;
    
    success = ~isempty(num) && isa(num,'numeric') && isscalar(num) && isreal(num) && isfinite(num);
end