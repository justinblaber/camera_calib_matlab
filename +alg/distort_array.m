function array_d = distort_array(array, f_p_p_d2p_p, opts)

    % Get distorted pixel coordinates
    [y_p_ds, x_p_ds] = alg.ndgrid_bb(alg.bb_array(array));

    % Transform coordinates from distorted pixels to pixels - this is
    % slow...
    p_ps = f_p_p_d2p_p([x_p_ds(:) y_p_ds(:)]);

    % Resample
    array_d = alg.interp_array(array, p_ps, opts.distort_array_interp);
    array_d = reshape(array_d, size(array));
end
