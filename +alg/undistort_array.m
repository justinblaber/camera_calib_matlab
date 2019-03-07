function array = undistort_array(array_d, f_p_p2p_p_d, opts)

    % Get pixel coordinates
    [y_ps, x_ps] = alg.ndgrid_bb(alg.bb_array(array_d));

    % Transform coordinates from pixels to distorted pixels
    p_p_ds = f_p_p2p_p_d([x_ps(:) y_ps(:)]);

    % Resample
    array = alg.interp_array(array_d, p_p_ds, opts.undistort_array_interp);
    array = reshape(array, size(array_d));
end
