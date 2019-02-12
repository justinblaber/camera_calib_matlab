function array = undistort_array(array_d, f_p_p2p_p_d, a, d, opts)
    % Undistorts array.
    %
    % Inputs:
    %   array_d - array; array containing distorted image
    %   f_p_p2p_p_d - function handle; describes the mapping between
    %       pixel coordinates and distorted pixel coordinates.
    %   a - array; 3x1 array containing:
    %       [alpha; x_o; y_o]
    %   d - array; Mx1 array of distortion coefficients
    %   opts - struct;
    %       .undistort_array_interp - string; type of interpolation to use
    %
    % Outputs:
    %   array - array; array containing undistorted image

    % Get pixel coordinates
    [y_ps, x_ps] = alg.ndgrid_bb(alg.bb_array(array_d));

    % Transform coordinates from pixels to distorted pixels
    p_p_ds = alg.p_p2p_p_d([x_ps(:) y_ps(:)], ...
                           f_p_p2p_p_d, ...
                           a, ...
                           d);

    % Resample
    array = alg.interp_array(array_d, p_p_ds, opts.undistort_array_interp);
    array = reshape(array, size(array_d));
end
