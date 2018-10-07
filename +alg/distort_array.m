function array_d = distort_array(array,xfm_p_bar2p_d,dxfm_p_bar2p_d_dx_p_bar,dxfm_p_bar2p_d_dy_p_bar,A,d,opts)
    % Distorts array. 
    %
    % Since points are optimized non-linearly, this function is pretty
    % slow. It should only be used for debugging purposes for now.
    %
    % Inputs:
    %   array - array; array containing undistorted image
    %   xfm_p_bar2p_d - function handle; describes mapping between ideal 
    %       pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    %   dxfm_p_bar2p_d_dx_p_bar - function handle; derivative of 
    %       xfm_p_bar2p_d wrt x_p_bar
    %   dxfm_p_bar2p_d_dy_p_bar - function handle; derivative of 
    %       xfm_p_bar2p_d wrt y_p_bar
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

    % Get distorted pixel coordinates
    [y_p_ds, x_p_ds] = ndgrid(1:size(array,1),1:size(array,2));
    
    % Transform coordinates from distorted pixels to ideal pixels - this is
    % slow...
    p_ps = alg.xfm_p_d2p(xfm_p_bar2p_d, ...
                         dxfm_p_bar2p_d_dx_p_bar, ...
                         dxfm_p_bar2p_d_dy_p_bar, ...
                         [x_p_ds(:) y_p_ds(:)], ... % Use distorted points for initial guess
                         [x_p_ds(:) y_p_ds(:)], ...
                         A, ...
                         d, ...
                         opts);
    
    % Resample
    array_d = alg.interp_array(array, ...
                               p_ps, ...
                               opts.distort_array_interp);
    array_d = reshape(array_d,size(array));
end