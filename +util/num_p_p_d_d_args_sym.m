function num_args_d = num_p_p_d_d_args_sym(sym_p_p_d)
    % Returns number of distortion parameters given a symbolic function
    %
    % Inputs:
    %   sym_p_p_d - symbolic function; describes mapping between ideal 
    %       pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    %
    % Outputs:
    %   num_args_d - int; number of distortion arguments
    
    num_args_d = numel(argnames(sym_p_p_d)) - 5;
end
