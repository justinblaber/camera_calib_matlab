function array = undistort_array(array_d,xfm_p_bar2p_d,A,d,opts)
    % Undistorts array.
    %
    % Inputs:
    %   array_d - array; array containing distorted image
    %   f_p_p_d - function handle; describes mapping between ideal 
    %       pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %   d - array; Mx1 array of distortion coefficients corresponding to
    %       input symbolic function
    %   opts - struct;
    %       .undistort_array_interp - string; type of interpolation to use
    % 
    % Outputs:
    %   array - array; array containing undistorted image

    % Get ideal pixel coordinates
    [y_ps, x_ps] = ndgrid(1:size(array_d,1),1:size(array_d,2));
    
    % Transform coordinates from ideal pixels to distorted pixels
    p_p_ds = alg.xfm_p2p_d(xfm_p_bar2p_d, ...
                           [x_ps(:) y_ps(:)], ...
                           A, ...
                           d);
    
    % Resample
    array = alg.interp_array(array_d, ...
                             p_p_ds, ...
                             opts.undistort_array_interp);
    array = reshape(array,size(array_d));
end