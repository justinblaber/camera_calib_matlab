function test_p_cb_w
    % Set options
    opts.num_targets_height = 2;
    opts.num_targets_width = 4;
    opts.target_spacing = 2;

    p_cb_ws = alg.p_cb_w(opts);

    % Assert
    assert(isequal(p_cb_ws, [0 0;
                             0 2;
                             2 0;
                             2 2;
                             4 0;
                             4 2;
                             6 0;
                             6 2]));
end
