function test_p_fp_w
    % Set options
    opts.num_targets_height = 2;
    opts.num_targets_width = 4;
    opts.target_spacing = 2;
    opts.height_fp = 5;
    opts.width_fp = 10;

    p_fp_ws = alg.p_fp_w(opts);

    % Assert
    assert(isequal(p_fp_ws, [-2.0 -1.5;
                             -2.0  3.5;
                              8.0 -1.5;
                              8.0  3.5]));
end
