function test_p_cb_w
    % Set options
    opts.height_fp = 5;
    opts.width_fp = 10;
    opts.num_targets_height = 2;
    opts.num_targets_width = 4;
    opts.target_spacing = 2;

    [p_cb_ws, p_fp_ws] = alg.p_cb_w(opts);

    % Assert
    assert(isequal(p_cb_ws, [0 0;
                            0 2;
                            2 0;
                            2 2;
                            4 0;
                            4 2;
                            6 0;
                            6 2]));
    assert(isequal(p_fp_ws, [-2.0 -1.5;
                            -2.0  3.5;
                             8.0 -1.5;
                             8.0  3.5]));
end
