function array_d = distort_array(array, obj_distortion, A, d, opts)
    % Distorts array.
    %
    % Since points are optimized non-linearly, this function is pretty
    % slow.
    %
    % Inputs:
    %   array - array; array containing undistorted image
    %   obj_distortion - class.distortion; describes the mapping between
    %       pixel coordinates and distorted pixel coordinates.
    %   A - array; 3x3 camera matrix
    %   d - array; Mx1 array of distortion coefficients
    %   opts - struct;
    %       .distort_array_interp - string; type of interpolation to use
    %
    % Outputs:
    %   array_d - array; array containing distorted image

    % Get distorted pixel coordinates
    [y_p_ds, x_p_ds] = alg.ndgrid_bb(alg.bb_array(array));

    % Transform coordinates from distorted pixels to pixels - this is
    % slow...
    p_ps = obj_distortion.p_p_d2p_p([x_p_ds(:) y_p_ds(:)], ...
                                    [x_p_ds(:) y_p_ds(:)], ...
                                    A, ...
                                    d);

    % Resample
    array_d = alg.interp_array(array, p_ps, opts.distort_array_interp);
    array_d = reshape(array_d, size(array));
end
