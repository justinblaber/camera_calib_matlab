function validate_p_p_d_sym_args(sym_p_p_d)
    % Makes sure input symbolic function is a valid distortion function
    %
    % Inputs:
    %   sym_p_p_d - symbolic function; describes mapping between ideal 
    %       pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    %
    % Outputs:
    %   none
    
    args = argnames(sym_p_p_d);
    args_str = arrayfun(@char,args,'UniformOutput',false);
    if ~strcmp(args_str(1:5),{'x_p_bar','y_p_bar','a','x_o','y_o'})
        error('Invalid symbolic distortion function; must start with "x_p_bar,y_p_bar,a,x_o,y_o"');
    end
end