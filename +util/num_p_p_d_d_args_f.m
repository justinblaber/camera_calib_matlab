function num_args_d = num_p_p_d_d_args_f(f_p_p_d)
    % Returns number of distortion parameters given a function handle
    %
    % Inputs:
    %   f_p_p_d - function handle; describes mapping between ideal 
    %       pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    %
    % Outputs:
    %   num_args_d - int; number of distortion arguments
    
    num_args_d = nargin(f_p_p_d) - 5;
end
