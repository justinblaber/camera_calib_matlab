function array_d = distort_array(array,f_p_p2p_p_d,f_dp_p_d_dx_p,f_dp_p_d_dy_p,a,d,opts)
    % Distorts array. 
    %
    % Since points are optimized non-linearly, this function is pretty 
    % slow.
    %
    % Inputs:
    %   array - array; array containing undistorted image
    %   f_p_p2p_p_d - function handle; describes the mapping between 
    %       pixel coordinates and distorted pixel coordinates.
    %   f_dp_p_d_dx_p - function handle; derivative of p_p2p_p_d wrt x_p
    %   f_dp_p_d_dy_p - function handle; derivative of p_p2p_p_d wrt y_p
    %   a - array; 3x1 array containing:
    %       [alpha; x_o; y_o]
    %   d - array; Mx1 array of distortion coefficients
    %   opts - struct;
    %       .p_p_d2p_p_it_cutoff - int; max number of iterations performed
    %           for transformation from distorted pixel coordinates to 
    %           pixel coordinates.
    %       .p_p_d2p_p_norm_cutoff - scalar; cutoff for norm of difference
    %           of parameter vector for transformation from distorted 
    %           pixel coordinates to pixel coordinates.
    %       .distort_array_interp - string; type of interpolation to use
    % 
    % Outputs:
    %   array_d - array; array containing distorted image
    
    % Get distorted pixel coordinates
    [y_p_ds, x_p_ds] = ndgrid(1:size(array,1),1:size(array,2));
    
    % Transform coordinates from distorted pixels to pixels - this is
    % slow...
    p_ps = alg.p_p_d2p_p([x_p_ds(:) y_p_ds(:)], ...
                         [x_p_ds(:) y_p_ds(:)], ... % Use distorted points for initial guess
                         f_p_p2p_p_d, ...
                         f_dp_p_d_dx_p, ...
                         f_dp_p_d_dy_p, ...                         
                         a, ...
                         d, ...
                         opts);
    
    % Resample
    array_d = alg.interp_array(array, ...
                               p_ps, ...
                               opts.distort_array_interp);
    array_d = reshape(array_d,size(array));
end