function array = apply_inv_p_p_d_array(array_d,f_p_p_d,A,d,opts)
    % Undoes distortion on image array given a function handle to
    % distortion function and intrinsic parameters.
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

    % Get array coordinates
    [y_p, x_p] = ndgrid(1:size(array_d,1),1:size(array_d,2));
    
    % Apply transform to coordinates
    p_p_d = alg.apply_p_p_d_f(f_p_p_d,[x_p(:) y_p(:)],A,d);
    
    % Resample
    array = alg.interp_array(array_d,p_p_d,opts.undistort_array_interp);
    array = reshape(array,size(array_d));
end