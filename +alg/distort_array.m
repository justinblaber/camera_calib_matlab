function array_d = distort_array(array,f_p_p_bar2p_p_d,f_dp_p_d_dx_p_bar,f_dp_p_d_dy_p_bar,A,d,opts)
    % Distorts array. 
    %
    % Since points are optimized non-linearly, this function is pretty
    % slow.
    %
    % Inputs:
    %   array - array; array containing undistorted image
    %   f_p_p_bar2p_p_d - function handle; describes mapping between ideal 
    %       pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    %   f_dp_p_d_dx_p_bar - function handle; derivative of p_p_bar2p_p_d 
    %       wrt x_p_bar
    %   f_dp_p_d_dy_p_bar - function handle; derivative of p_p_bar2p_p_d 
    %       wrt y_p_bar
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %   d - array; Mx1 array of distortion coefficients
    %   opts - struct;
    %       .distort_array_interp - string; type of interpolation to use
    % 
    % Outputs:
    %   array_d - array; array containing distorted image

    % Validate inputs
    util.validate_f_p_p_bar2p_p_d(f_p_p_bar2p_p_d);
    util.validate_f_p_p_bar2p_p_d(f_dp_p_d_dx_p_bar);
    util.validate_f_p_p_bar2p_p_d(f_dp_p_d_dy_p_bar);
    util.validate_A(A); 
    
    % Get distorted pixel coordinates
    [y_p_ds, x_p_ds] = ndgrid(1:size(array,1),1:size(array,2));
    
    % Transform coordinates from distorted pixels to ideal pixels - this is
    % slow...
    p_ps = alg.p_p_d2p_p([x_p_ds(:) y_p_ds(:)], ...
                         [x_p_ds(:) y_p_ds(:)], ... % Use distorted points for initial guess
                         f_p_p_bar2p_p_d, ...
                         f_dp_p_d_dx_p_bar, ...
                         f_dp_p_d_dy_p_bar, ...                         
                         A, ...
                         d, ...
                         opts);
    
    % Resample
    array_d = alg.interp_array(array, ...
                               p_ps, ...
                               opts.distort_array_interp);
    array_d = reshape(array_d,size(array));
end