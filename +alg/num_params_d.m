function num_params_d = num_params_d(f)
    % Returns number of distortion parameters, given a "distortion
    % function"
    %
    % Inputs:
    %   f - symbolic function or function handle; "distortion function"
    % 
    % Outputs:
    %   num_params_d - int; number of distortion parameters in "distortion
    %       function"
    
    % Get number of arguments
    switch class(f)
        case 'symfun'            
            num_args = numel(argnames(f));
        case 'function_handle'
            num_args = nargin(f);
        otherwise
            error('Input must be a symbolic function or function handle');
    end
    
    % Get number of distortion parameters
    num_params_d = num_args - 5;
end