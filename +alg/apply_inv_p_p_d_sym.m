function p_p = apply_inv_p_p_d_sym(sym_p_p_d,p_p_init,p_p_d,A,d,opts)
    % Computes inverse of distortion function given a symbolic function
    %
    % Inputs:
    %   sym_p_p_d - symbolic function; describes mapping between ideal 
    %       pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    %   p_p_init - array; Nx2 array of initial guess of ideal points
    %   p_p_d - array; Nx2 array of distorted points
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %   d - array; Mx1 array of distortion coefficients corresponding to
    %       input symbolic function
    %   opts - struct;
    %       .inv_p_p_d_it_cutoff - int; number of iterations performed
    %           for inverse distortion refinement
    %       .inv_p_p_d_norm_cutoff - scalar; cutoff for norm of 
    %           difference of parameter vector for inverse distortion
    %           refinement
    % 
    % Outputs:
    %   p_p - array; Nx2 array of optimized ideal points
    
    % Validate symbolic function arguments
    util.validate_p_p_d_sym_args(sym_p_p_d);
    
    % Get derivatives WRT x_p_bar and y_p_bar
    sym_dp_p_d_dp_p_bar = alg.diff_vars(sym_p_p_d,{'x_p_bar','y_p_bar'});
    
    % Convert symbolic functions to function handles for improved speed
    f_p_p_d = matlabFunction(sym_p_p_d);
    f_dp_p_d_dp_p_bar = matlabFunction(sym_dp_p_d_dp_p_bar);
    
    % Get inverse using function handles
    p_p = alg.apply_inv_p_p_d_f(f_p_p_d, ...
                                f_dp_p_d_dp_p_bar, ...
                                p_p_init, ...
                                p_p_d, ...
                                A, ...
                                d, ...
                                opts);
end