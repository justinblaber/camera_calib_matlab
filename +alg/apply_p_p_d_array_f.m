function array_d = apply_p_p_d_array_f(array,f_p_p_d,f_dp_p_d_dx_p_bar,f_dp_p_d_dy_p_bar,A,d,opts)
    % Applies distortion on image array given a function handle to
    % distortion function and intrinsic parameters. 
    %
    % Since points are optimized non-linearly, this function is pretty
    % slow. It should only be used for debugging purposes for now.
    %
    % Inputs:
    %   array - array; array containing undistorted image
    %   f_p_p_d - function handle; describes mapping between ideal 
    %       pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    %   f_dp_p_d_dx_p_bar - function handle; derivative of f_p_p_d wrt
    %       x_p_bar
    %   f_dp_p_d_dy_p_bar - function handle; derivative of f_p_p_d wrt
    %       y_p_bar
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %   d - array; Mx1 array of distortion coefficients corresponding to
    %       input symbolic function
    %   opts - struct;
    %       .apply_p_p_d_array_interp - string; type of interpolation
    % 
    % Outputs:
    %   array_d - array; array containing distorted image

    % Get array coordinates
    [y_p_d, x_p_d] = ndgrid(1:size(array,1),1:size(array,2));
    
    % Apply inverse transform to coordinates - this is slow...
    p_p = alg.apply_inv_p_p_d_f(f_p_p_d, ...
                                f_dp_p_d_dx_p_bar, ...
                                f_dp_p_d_dy_p_bar, ...
                                [x_p_d(:) y_p_d(:)], ... % Use distorted points for initial guess
                                [x_p_d(:) y_p_d(:)], ...
                                A, ...
                                d, ...
                                opts);
    
    % Resample
    array_d = alg.interp_array(array,p_p,opts.apply_p_p_d_array_interp);
    array_d = reshape(array_d,size(array));
end