function array_n = normalize_array(array)
    % Takes an input array and normalizes it between 0 and 1.
    %
    % Inputs:
    %   array - array; MxN array
    %
    % Outputs:
    %   array_n - array; MxN normalized array
    
    % NOTE: I think I will not handle Infs, NaNs or dividing by zero. I
    % think most algorithms that take in NaN(s) should just return NaN(s)
    % so you don't need to handle exceptions.
    
    % Get min and max of array
    min_array = min(array(:));
    max_array = max(array(:));
    
    % Normalize array
    array_n = (array-min_array)./(max_array-min_array);
end