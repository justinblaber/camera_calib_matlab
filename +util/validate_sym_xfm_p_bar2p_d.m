function validate_xfm_p_bar2p_d_args_sym(xfm_p_bar2p_d_sym)
    % Makes sure input symbolic function is a valid distortion function
    %
    % Inputs:
    %   xfm_p_bar2p_d_sym - symbolic function; describes mapping between ideal 
    %       pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    %
    % Outputs:
    %   none
    
    args = argnames(xfm_p_bar2p_d_sym);
    args_str = arrayfun(@char,args,'UniformOutput',false);
    if ~strcmp(args_str(1:5),{'x_p_bar','y_p_bar','a','x_o','y_o'})
        error('Invalid symbolic distortion function; must start with "x_p_bar,y_p_bar,a,x_o,y_o"');
    end
end