function test_refine_ring_point
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load conic data
    load(fullfile(tests_path, 'data', 'refine_ring_point.mat'));

    opts.target_optimization = 'opencv';
    [p_cb_p, cov_cb_p] = alg.refine_ring_point(p_cb_p_init, ...
                                               boundary_p_center, ...
                                               array_cb, ...
                                               array_cb_dx, ...
                                               array_cb_dy, ...
                                               opts);

    %{
    % Plot example
    f = figure;
    imshow(array_cb, []);
    hold on;
    plot(boundary_p_center([1 2 3 4 1], 1) + p_cb_p_init(1), boundary_p_center([1 2 3 4 1], 2) + p_cb_p_init(2), '-r');
    plot(p_cb_p_init(1), p_cb_p_init(2), 'rs');
    plot(p_cb_p(1), p_cb_p(2), 'gs');
    close(f);
    %}

    % Assert
    assert(all(all(abs(p_cb_p - 1.0e+02 * [1.283957357300145   5.129550364241935]) < 1e-4)));
    assert(all(all(abs(cov_cb_p - [ 0.001072118776111  -0.000012856422508;
                                   -0.000012856422508   0.000981387442459]) < 1e-4)));
end
