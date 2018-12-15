function array = normalize_array(array, type)
    % Takes an input array and normalizes it between 0 and 1.
    %
    % Inputs:
    %   array - array; MxN array
    %   type - string; type of normalization to perform
    %
    % Outputs:
    %   array - array; MxN normalized array
    
    % NOTE: I think I will not handle Infs, NaNs or dividing by zero. I
    % think most algorithms that take in NaN(s) should just return NaN(s)
    % so you don't need to handle exceptions.
    
    switch type
        case 'min-max'
            % Scales array between 0 and 1
            min_array = min(array(:));
            max_array = max(array(:));
            array = (array-min_array)./(max_array-min_array);            
        case 'mean-norm'
            % Subtracts mean, then divides by norm
            array = array - mean(array(:));
            array = array./norm(array);
        otherwise
            error(['Normalization type: "' type '" not recognized!'])
    end
end