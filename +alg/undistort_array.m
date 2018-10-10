function array = undistort_array(array_d,f_p_p2p_p_d,A,d,opts)
    % Undistorts array.
    %
    % Inputs:
    %   array_d - array; array containing distorted image
    %   f_p_p2p_p_d - function handle; describes the mapping between 
    %       pixel coordinates and distorted pixel coordinates.
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %   d - array; Mx1 array of distortion coefficients
    %   opts - struct;
    %       .undistort_array_interp - string; type of interpolation to use
    % 
    % Outputs:
    %   array - array; array containing undistorted image

    % Validate inputs
    util.validate_f_p_p2p_p_d(f_p_p2p_p_d);
    util.validate_A(A); 
    
    % Get pixel coordinates
    [y_ps, x_ps] = ndgrid(1:size(array_d,1),1:size(array_d,2));
    
    % Transform coordinates from pixels to distorted pixels
    p_p_ds = alg.p_p2p_p_d([x_ps(:) y_ps(:)], ...
                           f_p_p2p_p_d, ...                           
                           A, ...
                           d);
    
    % Resample
    array = alg.interp_array(array_d, ...
                             p_p_ds, ...
                             opts.undistort_array_interp);
    array = reshape(array,size(array_d));
end